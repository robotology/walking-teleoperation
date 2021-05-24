/**
 * @file VirtualizerModule.cpp
 * @authors  Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 *           Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#define _USE_MATH_DEFINES
#include <cmath>

// YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/dev/IAxisInfo.h>

#include "Utils.hpp"
#include "VirtualizerModule.hpp"

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
    m_dT = rf.check("period", yarp::os::Value(0.1)).asDouble();

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

    if (!YarpHelper::getStringFromSearchable(rf, "rpcWalkingPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_rpcPort.open("/" + getName() + portName))
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
    m_rpcPort.close();
    m_robotOrientationPort.close();
    m_playerOrientationPort.close();
    m_rpcServerPort.close();

    // deallocate memory
    delete m_cvirtDeviceID;

    m_headDevice.close();
    m_encodersInterface = nullptr;
    m_controlModeInterface = nullptr;

    return true;
}

bool VirtualizerModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    // get data from virtualizer
    double playerYaw;
    playerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());

    playerYaw *= 360.0f;
    playerYaw = playerYaw * M_PI / 180;
    playerYaw = Angles::normalizeAngle(playerYaw);

    yInfo() << "Current player yaw: " << playerYaw;

    if (std::fabs(Angles::shortestAngularDistance(playerYaw, m_oldPlayerYaw)) > 0.15)
    {
        yError() << "Virtualizer misscalibrated or disconnected";
        return false;
    }

    // get the player speed
    double speedData = threshold((double)(m_cvirtDeviceID->GetMovementSpeed()), m_speedDeadzone);

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
        double filteredVelocity = threshold(filteredRingVelocity(newVelocity), m_velocityDeadzone);
        y = -m_velocityScaling * filteredVelocity; //The ring has the z axis pointing downward
    }
    else
    {
        // get the robot orientation
        yarp::sig::Vector* tmp = m_robotOrientationPort.read(false);
        if (tmp != NULL)
        {
            auto vector = *tmp;
            m_robotYaw = -Angles::normalizeAngle(vector[0]);
        }

        // error between the robot orientation and the player orientation
        double angularError = threshold(Angles::shortestAngularDistance(m_robotYaw, playerYaw), m_angleDeadzone);
        y = -m_scale_Y * angularError; // because the virtualizer orientation value is CCW, therefore we put "-" to
        // make it CW, same as the robot world.
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
    yarp::os::Bottle cmd, outcome;
    cmd.addString("setGoal");
    cmd.addDouble(x);
    cmd.addDouble(y);
    m_rpcPort.write(cmd, outcome);


    m_oldPlayerYaw = playerYaw;

    // send the orientation of the player
    yarp::sig::Vector& playerOrientationVector = m_playerOrientationPort.prepare();
    playerOrientationVector.clear();
    playerOrientationVector.push_back(playerYaw);
    m_playerOrientationPort.write();

    return true;
}

void VirtualizerModule::resetPlayerOrientation()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_cvirtDeviceID->ResetPlayerOrientation();

    m_oldPlayerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());
    m_oldPlayerYaw *= 360.0f;
    m_oldPlayerYaw = m_oldPlayerYaw * M_PI / 180;
    m_oldPlayerYaw = Angles::normalizeAngle(m_oldPlayerYaw);

    m_movingAverage.clear();
    m_movingAverage.resize(m_movingAverageWindowSize, 0.0);
    return;
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
