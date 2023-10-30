// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#define _USE_MATH_DEFINES
#include <cmath>

// YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/dev/IAxisInfo.h>

#include "Utils.hpp"
#include "VirtualizerModule.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

bool VirtualizerModule::configureVirtualizer()
{
    // try to connect to the virtualizer
    int maxAttempt = 5;
    for (int i = 0; i < maxAttempt; i++)
    {
        m_cvirtDeviceID = CybSDK::Virt::FindDevice();
        if (m_cvirtDeviceID != nullptr)
        {
            if (!m_cvirtDeviceID->Open())
            {
                yError() << "[configureVirtualizer] Unable to open the device";
                return false;
            }

            return true;
        }
        // wait one millisecond
        yarp::os::Time::delay(0.001);
    }

    yError() << "[configureVirtualizer] I'm not able to configure the virtualizer";
    return false;
}

bool VirtualizerModule::configureRingVelocity(const yarp::os::Bottle &ringVelocityGroup)
{
    if (ringVelocityGroup.isNull())
    {
        yError() << "y_use_ring_velocity is set to true but the RING_VELOCITY group is empty.";
        return false;
    }

    if (!YarpHelper::getUnsignedIntFromSearchable(ringVelocityGroup, "moving_average_window", m_movingAverageWindowSize))
    {
        yError() << "Failed to read moving_average_window";
        return false;
    }

    m_movingAverage.clear();
    m_movingAverage.resize(m_movingAverageWindowSize, 0.0);

    if (!YarpHelper::getDoubleFromSearchable(ringVelocityGroup, "velocity_deadzone", m_velocityDeadzone))
    {
        yError() << "Failed to read velocity_deadzone";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(ringVelocityGroup, "velocity_scaling", m_velocityScaling))
    {
        yError() << "Failed to read velocity_scaling";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(ringVelocityGroup, "angle_threshold_still_rad", m_angleThresholdOperatorStill))
    {
        yError() << "Failed to read angle_threshold_still_rad";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(ringVelocityGroup, "angle_threshold_moving_rad", m_angleThresholdOperatorMoving))
    {
        yError() << "Failed to read angle_threshold_moving_rad";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(ringVelocityGroup, "time_threshold_moving_s", m_operatorStillTimeThreshold))
    {
        yError() << "Failed to read time_threshold_moving_s";
        return false;
    }

    m_operatorCurrentStillAngle = 0.0;
    m_operatorStillTime = -1.0;
    m_operatorMoving = true;

    if (m_angleThresholdOperatorMoving < m_angleThresholdOperatorStill)
    {
        yError() << "The angle threshold to consider the operator moving needs to be greater than the one to consider him still.";
        return false;
    }

    if (m_operatorStillTimeThreshold < 0.0)
    {
        yError() << "The time threshold to consider the operator still needs to be greater than 0.";
        return false;
    }

    if (!YarpHelper::getBooleanFromSearchable(ringVelocityGroup, "use_sign_only", m_useVelocitySignOnly))
    {
        yError() << "Failed to read use_sign_only";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(ringVelocityGroup, "jammed_moving_time_s", m_jammedMovingTime))
    {
        yError() << "Failed to read jammed_moving_time_s";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(ringVelocityGroup, "jammed_moving_robot_angle_rad", m_jammedMovingRobotAngle))
    {
        yError() << "Failed to read jammed_moving_robot_angle_rad";
        return false;
    }

    if (m_jammedMovingTime > 0 && m_jammedMovingRobotAngle > 0)
    {
        yWarning() << "Both jammed_moving_time_s and jammed_moving_robot_angle_rad are greater than 0."
                   << "Only the robot condition will be used, unless the robot angle reading is not valid";
    }

    m_jammedStartTime = -1.0;
    m_jammedRobotStartAngle = 0.0;
    m_jammedRobotStartAngleValid = false;
    m_isJammed = false;
    m_jammedValue = 0.0;
    m_jammedRobotOnce = false;

    return true;
}

bool VirtualizerModule::configureTransformServer(const yarp::os::Bottle &tfGroup)
{
    if (!YarpHelper::getStringFromSearchable(tfGroup, "root_frame_name", m_tfRootFrame))
    {
        yError() << "Failed while reading root_frame_name parameter.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(tfGroup, "frame_name", m_tfFrameName))
    {
        yError() << "Failed while reading frame_name parameter.";
        return false;
    }

    //opening tf client
    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", tfGroup.check("transform_server_device", yarp::os::Value("frameTransformClient")).asString());
    tfClientCfg.put("filexml_option",  tfGroup.check("transform_server_file", yarp::os::Value("ftc_yarp_only.xml")).asString());
    tfClientCfg.put("ft_client_prefix", tfGroup.check("transform_server_local", yarp::os::Value(getName() + "/tf")).asString());
    if (tfGroup.check("transform_server_remote"))
    {
        tfClientCfg.put("ft_server_prefix", tfGroup.find("transform_server_remote").asString());
    }
    tfClientCfg.put("local_rpc", "/" + getName() + "/tf/local_rpc");

    if (!m_tfDriver.open(tfClientCfg))
    {
        yError() << "Unable to open polydriver with the following options:" << tfClientCfg.toString();
        return false;
    }

    if (!m_tfDriver.view(m_tfPublisher) || m_tfPublisher == nullptr)
    {
        yError() << "Unable to view IFrameTransform interface.";
        return false;
    }

    m_tfMatrix.resize(4,4);
    m_tfMatrix.eye();

    return true;
}

bool VirtualizerModule::configureHeadControl(const yarp::os::Bottle &headControlGroup)
{
    if (headControlGroup.isNull())
    {
        yError() << "use_head_for_turning is set to true but the HEAD_CONTROL group is empty.";
        return false;
    }

    std::string robot;
    if (!YarpHelper::getStringFromSearchable(headControlGroup, "robot", robot))
    {
        yError() << "Failed while reading robot parameter.";
        return false;
    }

    std::string remoteControlBoard;
    if (!YarpHelper::getStringFromSearchable(headControlGroup, "remote_control_board", remoteControlBoard))
    {
        yError() << "Failed while reading remote_control_board parameter.";
        return false;
    }

    std::string neckYawName;
    if (!YarpHelper::getStringFromSearchable(headControlGroup, "neck_yaw_name", neckYawName))
    {
        yError() << "Failed while reading neck_yaw_name parameter.";
        return false;
    }

    if (!YarpHelper::getBooleanFromSearchable(headControlGroup, "yaw_axis_points_up", m_yawAxisPointsUp))
    {
        yError() << "Failed while reading yaw_axis_points_up parameter.";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(headControlGroup, "neck_yaw_scaling", m_neckYawScaling))
    {
        yError() << "Failed to read neck_yaw_scaling";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(headControlGroup, "neck_yaw_deadzone", m_neckYawDeadzone))
    {
        yError() << "Failed to read neck_yaw_deadzone";
        return false;
    }

    if (!YarpHelper::getBooleanFromSearchable(headControlGroup, "use_only_head_to_turn", m_useOnlyHeadForTurning))
    {
        yError() << "Failed while reading yaw_axis_points_up parameter.";
        return false;
    }

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/" + getName() + "/neckControlBoard");
    options.put("remote", "/" + robot + "/" + remoteControlBoard);

    if (!m_headDevice.open(options))
    {
        yError() << "Failed to open device to get neck encoders.";
        return false;
    }

    if (!m_headDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "Failed to retrieve encoders interface.";
        return false;
    }

    if (!m_headDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "Failed to retrieve control mode interface.";
        return false;
    }

    yarp::dev::IAxisInfo* axisInfo;

    if (!m_headDevice.view(axisInfo) || !axisInfo)
    {
        yError() << "Failed to retrieve axis info.";
        return false;
    }

    int numberOfAxes = 0;
    if (!m_encodersInterface->getAxes(&numberOfAxes))
    {
        yError() << "Failed to get the number of axes";
        return false;
    }

    int i = 0;
    m_neckYawAxisIndex = -1;
    std::string axisName;

    while ((i < numberOfAxes) && (m_neckYawAxisIndex < 0))
    {
        if (!axisInfo->getAxisName(i, axisName))
        {
            yError() << "Failed to get the name of the axis with index " << i << ".";
            return false;
        }

        if (axisName == neckYawName)
        {
            m_neckYawAxisIndex = i;
        }
        ++i;
    }

    if (m_neckYawAxisIndex < 0)
    {
        yError() << "Failed to find joint named " << neckYawName << ".";
        return false;
    }

    return true;
}

bool VirtualizerModule::configure(yarp::os::ResourceFinder& rf)
{
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[configure] Empty configuration.";
        return false;
    }

    // get the period
    m_dT = rf.check("period", yarp::os::Value(0.1)).asFloat64();

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    // set scales for walking
    if (!YarpHelper::getDoubleFromSearchable(rf, "scale_X", m_scale_X))
    {
        yError() << "[configure] Unable to get scale_X from a searchable";
        return false;
    }
    if (!YarpHelper::getDoubleFromSearchable(rf, "scale_Y", m_scale_Y))
    {
        yError() << "[configure] Unable to get scale_Y from a searchable";
        return false;
    }

    // set deadzone
    if (!YarpHelper::getDoubleFromSearchable(rf, "angle_deadzone", m_angleDeadzone))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(rf, "speed_deadzone", m_speedDeadzone))
    {
        yError() << "Failed to read speed_deadzone";
        return false;
    }

    // open ports
    std::string portName;
    if (!YarpHelper::getStringFromSearchable(rf, "playerOrientationPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }

    if (!m_playerOrientationPort.open("/" + getName() + portName))
    {
        yError() << "[configure] " << portName << " port already open.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "robotOrientationPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }

    if (!m_robotOrientationPort.open("/" + getName() + portName))
    {
        yError() << "[configure] " << portName << " port already open.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "goalWalkingPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_robotGoalPort.open("/" + getName() + portName))
    {
        yError() << "[configure] " << portName << " port already open.";
        return false;
    }

    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcServerPort);
    if (!m_rpcServerPort.open(rpcPortName))
    {
        yError() << "[configure] Could not open" << rpcPortName << "RPC port.";
        return false;
    }

    m_useRingVelocity = rf.check("y_use_ring_velocity", yarp::os::Value(false)).asBool();

    if (m_useRingVelocity)
    {
        if (!configureRingVelocity(rf.findGroup("RING_VELOCITY")))
        {
            yError() << "Failed to configure ring velocity control.";
            return false;
        }
    }

    m_useHeadForTurning = rf.check("use_head_for_turning", yarp::os::Value(false)).asBool();

    if (m_useHeadForTurning)
    {
        if (!configureHeadControl(rf.findGroup("HEAD_CONTROL")))
        {
            yError() << "Failed to configure head control.";
            return false;
        }
    }

    m_useTf = rf.check("use_transform_server", yarp::os::Value(false)).asBool();
    if (m_useTf)
    {
        if (!configureTransformServer(rf.findGroup("TF")))
        {
            yError() << "Failed to configure transform server.";
            return false;
        }
    }

    if (!configureVirtualizer())
    {
        yError() << "[configure] Unable to configure the virtualizer";
        return false;
    }

    // remove me!!!
    // this is because the virtualizer is not ready
    yarp::os::Time::delay(0.5);

    // reset player orientation
    m_cvirtDeviceID->ResetPlayerOrientation();

    m_cvirtDeviceID->ResetPlayerHeight();

    // reset some quanties
    m_robotYaw = 0;
    m_oldPlayerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());
    m_oldPlayerYaw *= 360.0f;
    m_oldPlayerYaw = m_oldPlayerYaw * M_PI / 180;
    m_oldPlayerYaw = Angles::normalizeAngle(m_oldPlayerYaw);

    return true;
}

double VirtualizerModule::getPeriod()
{
    return m_dT;
}

bool VirtualizerModule::close()
{
    // close the ports
    m_robotGoalPort.close();
    m_robotOrientationPort.close();
    m_playerOrientationPort.close();
    m_rpcServerPort.close();

    // deallocate memory
    delete m_cvirtDeviceID;

    m_headDevice.close();
    m_encodersInterface = nullptr;
    m_controlModeInterface = nullptr;

    m_tfDriver.close();
    m_tfPublisher = nullptr;

    return true;
}

bool VirtualizerModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    // get data from virtualizer
    double playerYaw = getPlayerYaw();

    yInfo() << "Current player yaw: " << playerYaw;

    if (std::fabs(Angles::shortestAngularDistance(playerYaw, m_oldPlayerYaw)) > 0.15)
    {
        yWarning() << "Virtualizer misscalibrated or disconnected";
    }

    //get player height
    float playerHeightInCm;
    playerHeightInCm = m_cvirtDeviceID->GetPlayerHeight(); //It is relative to the initial height set ResetPlayerHeight, positive upward
    double playerHeighInM = playerHeightInCm / 100.0;

    // get the player speed
    double virtualizerSpeedData = (double)(m_cvirtDeviceID->GetMovementSpeed());
    double speedData = threshold(virtualizerSpeedData, m_speedDeadzone);

    double tmpSpeedDirection = (double)(m_cvirtDeviceID->GetMovementDirection());
    double speedDirection = 1.0; // set the speed direction to forward by default.

    if (std::abs(tmpSpeedDirection) < 0.01) // the "0" value means the user walking forward
        speedDirection = 1.0;
    else if (std::abs(tmpSpeedDirection + 1) < 0.01)  // the "-1" value means the user walking backward
    {
        speedDirection = -1.0;
    } else
    {
        yError() << "The virtualizer speed direction is not normal; speed direction value: "
                 << tmpSpeedDirection;
        return false;
    }

    double x = speedDirection * m_scale_X * speedData;
    double y = 0;
    if (m_useRingVelocity)
    {
        double newVelocity = Angles::shortestAngularDistance(m_oldPlayerYaw, playerYaw)/getPeriod();
        double filteredVelocity = 0.0;

        if (m_useVelocitySignOnly)
        {
            filteredVelocity = sign(filteredRingVelocity(newVelocity), m_velocityDeadzone);
        }
        else
        {
            filteredVelocity = threshold(filteredRingVelocity(newVelocity), m_velocityDeadzone);
        }

        //If the operator is moving and then remains close to a certain angle for a while, then we consider the operator still
        //if the operator is still, it needs to move of a certain angle before considering it moving.

        if (m_operatorMoving)
        {
            bool operatorIsNotRotating
                = std::abs(Angles::shortestAngularDistance(m_operatorCurrentStillAngle, playerYaw))
                  < m_angleThresholdOperatorStill;
            bool operatorIsNotWalking = std::abs(virtualizerSpeedData) < m_speedDeadzone;

            if (operatorIsNotRotating && operatorIsNotWalking)
            {
                yInfo() << "The operator seems fixed";
                if (m_operatorStillTime < 0)
                {
                    m_operatorStillTime = yarp::os::Time::now();
                }
                else if (yarp::os::Time::now() - m_operatorStillTime > m_operatorStillTimeThreshold) {
                    yInfo() << "The operator is fixed";
                    m_operatorMoving = false;
                }
            }
            else
            {
                m_operatorCurrentStillAngle = playerYaw;
                m_operatorStillTime = -1.0;
            }
        }
        else
        {
            double angleVariation = Angles::shortestAngularDistance(m_operatorCurrentStillAngle, playerYaw);
            if (std::abs(angleVariation) > m_angleThresholdOperatorMoving)
            {
                yInfo() << "The operator is moving";

                m_jammedStartTime = yarp::os::Time::now();

                if (m_jammedMovingRobotAngle > 0.0 && !m_jammedRobotOnce)
                {
                    m_jammedRobotOnce = true;
                    m_jammedRobotStartAngleValid = updateRobotYaw();
                    m_jammedRobotStartAngle = m_robotYaw;

                    if (!m_jammedRobotStartAngleValid)
                    {
                        yWarning() << "The robot angle is not valid.";
                    }
                }

                if (angleVariation > 0)
                {
                    m_jammedValue = 1.0;
                }
                else
                {
                    m_jammedValue = -1.0;
                }
                m_isJammed = true;

                m_operatorMoving = true;
                m_operatorCurrentStillAngle = playerYaw;
                m_operatorStillTime = -1.0;
            }
        }

        if (m_isJammed)
        {
            if (m_jammedRobotStartAngleValid && updateRobotYaw())
            {
                if (std::abs(Angles::shortestAngularDistance(m_jammedRobotStartAngle, m_robotYaw)) >= m_jammedMovingRobotAngle)
                {
                    m_isJammed = false;
                }
            }
            else if (yarp::os::Time::now() - m_jammedStartTime >= m_jammedMovingTime)
            {
                m_isJammed = false;
            }
        }

        if (m_isJammed)
        {
            filteredVelocity = m_jammedValue;
        }
        else if (!m_operatorMoving)
        {
            filteredVelocity = 0.0;
        }

        y = -m_velocityScaling * filteredVelocity; //The ring has the z axis pointing downward
    }
    else
    {
        // get the robot orientation
        updateRobotYaw();

        // error between the robot orientation and the player orientation
        double angularError = threshold(Angles::shortestAngularDistance(m_robotYaw, playerYaw), m_angleDeadzone);
        y = -m_scale_Y * angularError; // because the virtualizer orientation value is CW, therefore we put "-" to
        // make it CCW, same as the robot world.
    }

    if (m_useHeadForTurning)
    {
        if (std::fabs(speedData) > 0.0) //We use the head for turning only if the operator is walking.
        {
            if (isNeckWorking())
            {
                if (m_useOnlyHeadForTurning)
                {
                    y = 0;
                }

                double neckSign = m_yawAxisPointsUp ? +1.0 : -1.0;

                double neckValue;

                if (m_encodersInterface->getEncoder(m_neckYawAxisIndex, &neckValue))
                {
                    double neckValueFiltered = threshold(neckValue, m_neckYawDeadzone);
                    y += neckSign * m_neckYawScaling * neckValueFiltered;
                }
                else
                {
                    yWarning() << "Failed to read neck yaw encoder.";
                }
            }
            else
            {
                yWarning() << "The head seems to have issues. Avoiding to use it to control the direction";
            }
        }
    }

    yInfo() << "speed (x,y): " << x << " , " << y;

    // send data to the walking module
    yarp::sig::Vector& goal = m_robotGoalPort.prepare();
    goal.clear();
    goal.push_back(x);
    goal.push_back(y);
    m_robotGoalPort.write();


    m_oldPlayerYaw = playerYaw;

    // send the orientation of the player
    yarp::sig::Vector& playerOrientationVector = m_playerOrientationPort.prepare();
    playerOrientationVector.clear();
    playerOrientationVector.push_back(playerYaw);
    m_playerOrientationPort.write();

    if (m_useTf)
    {
        Eigen::Matrix3d rotation = Eigen::AngleAxisd(-playerYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); //We consider the Z axis pointing upward, hence we invert the sign since the angle from the virtualizer is positive clockwise

        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                m_tfMatrix(i,j) = rotation(i,j);
            }
        }
        m_tfMatrix(2,3) = playerHeighInM; //Set the z position equal to the height

        m_tfPublisher->setTransform(m_tfFrameName, m_tfRootFrame, m_tfMatrix);
    }

    return true;
}

void VirtualizerModule::resetPlayerOrientation()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_cvirtDeviceID->ResetPlayerOrientation();

    m_oldPlayerYaw = getPlayerYaw();

    m_movingAverage.clear();
    m_movingAverage.resize(m_movingAverageWindowSize, 0.0);
    return;
}

void VirtualizerModule::resetPlayerHeight()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_cvirtDeviceID->ResetPlayerHeight();
}

void VirtualizerModule::forceStillAngle()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_operatorMoving = false;
    m_operatorCurrentStillAngle = getPlayerYaw();
    m_operatorStillTime = -1.0;

    m_isJammed = false;
    m_jammedRobotOnce = false;


    yInfo() << "Forced the operator still angle to" << m_operatorCurrentStillAngle;
}

double VirtualizerModule::threshold(const double &input, double deadzone)
{
    if (input >= 0)
    {
        if (input > deadzone)
            return input;
        else
            return 0.0;
    } else
    {
        if (input < -deadzone)
            return input;
        else
            return 0.0;
    }
}

double VirtualizerModule::sign(const double &input, double deadzone)
{
    if (std::abs(input) < deadzone)
    {
        return 0.0;
    }

    if (input > 0)
    {
        return +1.0;
    }

    return -1.0;
}

double VirtualizerModule::filteredRingVelocity(double newVelocity)
{
    m_movingAverage.pop_back();
    m_movingAverage.push_front(newVelocity);

    double summation = 0;

    for (const double& value : m_movingAverage)
    {
        summation += value;
    }

    return summation/m_movingAverage.size();
}

bool VirtualizerModule::isNeckWorking()
{
    if (m_controlModeInterface)
    {
        int mode;
        if (!m_controlModeInterface->getControlMode(m_neckYawAxisIndex, &mode))
        {
            yWarning() << "Failed to get control mode.";
            return false;
        }

        if ((mode == VOCAB_CM_IDLE) ||
            (mode == VOCAB_CM_NOT_CONFIGURED) ||
            (mode == VOCAB_CM_HW_FAULT) ||
            (mode == VOCAB_CM_UNKNOWN))
        {
            return false;
        }
    }
    else
    {
        yWarning() << "Control mode interface not working.";
        return false;
    }

    return true;
}

bool VirtualizerModule::updateRobotYaw()
{
    // get the robot orientation
    yarp::sig::Vector* tmp = m_robotOrientationPort.read(false);
    if (tmp == nullptr)
    {
        return false;
    }

    auto vector = *tmp;
    m_robotYaw = -Angles::normalizeAngle(vector[0]);
    return true;
}

double VirtualizerModule::getPlayerYaw()
{
    double playerYaw;
    playerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());

    playerYaw *= 360.0f; //The angle from the virtualizer is a number from 0 to 1
    playerYaw = playerYaw * M_PI / 180;
    playerYaw = Angles::normalizeAngle(playerYaw);

    return playerYaw;
}
