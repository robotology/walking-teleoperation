/**
 * @file RobotInterface.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <algorithm>
#include <iterator>
#include <limits>

// iDynTree
#include <iDynTree/Core/Utils.h>

// teleoperation
#include <ControlHelper.hpp>
#include <RobotInterface.hpp>
#include <Utils.hpp>

using namespace HapticGlove;

bool RobotInterface::configure(const yarp::os::Searchable& config,
                               const std::string& name,
                               const bool& rightHand,
                               const bool& isMandatory)
{
    m_rightHand = rightHand;
    m_logPrefix = "RobotInterface::";
    m_logPrefix += m_rightHand ? "RightHand:: " : "LeftHand:: ";

    m_isMandatory = isMandatory;

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    if (!YarpHelper::getVectorFromSearchable(config, "axis_list", m_actuatedAxisNames))
    {
        yError() << m_logPrefix << "unable to get  axis_list from the config file.";
        return false;
    }
    m_noActuatedAxis = m_actuatedAxisNames.size();

    if (!YarpHelper::getVectorFromSearchable(config, "all_axis_list", m_allAxisNames))
    {
        yError() << m_logPrefix << "unable to get  all_axis_list from the config file..";
        return false;
    }

    // // 5*3 analog sensors + 1 encoder thumb oppose + 1 encoder hand_finger
    if (!YarpHelper::getVectorFromSearchable(config, "joint_list", m_allJointNames))
    {
        yError() << m_logPrefix << "unable to get joint_list from the config file.";
        return false;
    }

    m_noAllAxis = m_allAxisNames.size();

    m_noAllJoints = m_allJointNames.size();

    // add the list of axis and associated analog sensors
    std::vector<std::string> analogList;
    if (!YarpHelper::getVectorFromSearchable(config, "analog_list", analogList))
    {
        yError() << m_logPrefix << "unable to get analog_list from config file.";
        return false;
    }

    m_noAnalogSensor = analogList.size();

    std::vector<bool> jointFeedbackIsAnalog(m_noAllJoints, false);
    for (size_t i = 0; i < m_allJointNames.size(); i++)
    {
        if (find(std::begin(analogList), std::end(analogList), m_allJointNames[i])
            != std::end(analogList))
        {
            jointFeedbackIsAnalog[i] = true;
        }
    }

    for (size_t axisIndex = 0; axisIndex < m_actuatedAxisNames.size(); axisIndex++)
    {
        std::vector<std::string> axisJointList; // related joints to the axisName

        if (!YarpHelper::getVectorFromSearchable(
                config, m_actuatedAxisNames[axisIndex], axisJointList))
        {
            yError() << m_logPrefix << "unable to find " << m_actuatedAxisNames[axisIndex]
                     << "as a key into config file.";
            return false;
        }

        // *it: joint name related to the axisName
        for (std::vector<std::string>::iterator it = axisJointList.begin();
             it != axisJointList.end();
             ++it)
        {
            JointInfo jointInfo;
            m_actuatedJointList.push_back(*it);

            auto elementJoint
                = std::find(std::begin(m_allJointNames), std::end(m_allJointNames), *it);
            if (elementJoint == std::end(m_allJointNames))
            {
                yError() << m_logPrefix << "unable to find " << *it
                         << " in joint vector list vector of strings.";
                return false;
            }
            size_t jointIndex = elementJoint - m_allJointNames.begin();

            jointInfo.useAnalog = jointFeedbackIsAnalog[jointIndex];
            jointInfo.scale = 1.0;

            if (jointInfo.useAnalog)
            {
                auto elementAnalog = std::find(std::begin(analogList), std::end(analogList), *it);
                if (elementAnalog == std::end(analogList))
                {
                    yError() << m_logPrefix << "the joint name (" << *it
                             << " )is not found in the analog list, but it should be an "
                                "analog sensor.";
                    return false;
                }
                jointInfo.index = elementAnalog - analogList.begin();

            } else
            {
                // if a joint does not have analog sensor to get its feedback, we use the associated
                // encoder value of the axis.
                jointInfo.index = axisIndex;

                // This parameter is used only when using axis encoders to compute the joint angle.
                // In this case, the encoder readouts are scaled to the robot joint values. This is
                // mainly useful when single axis encoder value is used to specify several joint
                // angle values due to the lack of sensors (e.g., l_hand_finger or r_hand_finger
                // associated joints).
                jointInfo.scale = 1.0 / axisJointList.size();
            }
            m_jointInfoMap.insert(std::make_pair(m_actuatedJointList.back(), jointInfo));
        }
    }

    m_noActuatedJoints = m_actuatedJointList.size();
    if (m_jointInfoMap.size() != m_noActuatedJoints)
    {
        yError() << m_logPrefix
                 << "the size of m_jointInfoMap should be equal to the actuated joint list.";
        return false;
    }

    m_analogJointsMinBoundaryDegree.resize(m_noAnalogSensor, 0.0);
    m_analogJointsMaxBoundaryDegree.resize(m_noAnalogSensor, 0.0);
    m_analogSensorsRawMinBoundary.resize(m_noAnalogSensor, 0.0);
    m_analogSensorsRawMaxBoundary.resize(m_noAnalogSensor, 0.0);

    // get the joints limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "analog_joints_min_boundary", m_analogJointsMinBoundaryDegree))
    {
        yError() << m_logPrefix << "unable to get the minimum boundary of the joints limits.";
        return false;
    }

    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "analog_joints_max_boundary", m_analogJointsMaxBoundaryDegree))
    {
        yError() << m_logPrefix << "unable to get the maximum boundary of the joints limits.";
        return false;
    }

    // get the sensors limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "analog_sensors_raw_min_boundary", m_analogSensorsRawMinBoundary))
    {
        yError() << m_logPrefix << "unable to get the minimum boundary of the joints limits.";
        return false;
    }

    // get the sensors limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "analog_sensors_raw_max_boundary", m_analogSensorsRawMaxBoundary))
    {
        yError() << m_logPrefix << "unable to get the maximum boundary of the joints limits.";
        return false;
    }
    bool tmp = (m_analogJointsMinBoundaryDegree.size() == m_noAnalogSensor)
               && (m_analogJointsMaxBoundaryDegree.size() == m_noAnalogSensor)
               && (m_analogSensorsRawMinBoundary.size() == m_noAnalogSensor)
               && (m_analogSensorsRawMaxBoundary.size() == m_noAnalogSensor);

    if (!tmp)
    {
        yError()
            << m_logPrefix
            << "analog joint min/max boundary or analog sensor raw value sizes are not correct.";
        return false;
    }

    m_sensorsRaw2DegreeScaling.resize(m_noAnalogSensor, 0.0);
    for (size_t i = 0; i < m_noAnalogSensor; i++)
    {
        m_sensorsRaw2DegreeScaling(i)
            = double(m_analogJointsMaxBoundaryDegree(i) - m_analogJointsMinBoundaryDegree(i))
              / double(m_analogSensorsRawMaxBoundary(i) - m_analogSensorsRawMinBoundary(i));
    }

    // Devices

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    if (!YarpHelper::getVectorFromSearchable(config, "remote_control_boards", iCubParts))
    {
        yError() << m_logPrefix
                 << "unable to get remote_control_boards vector of strings from config file.";
        return false;
    }

    // get all icub senosry parts from the resource finder
    std::string iCubSensorPart;

    if (!YarpHelper::getStringFromSearchable(config, "remote_sensor_boards", iCubSensorPart))
    {
        yError() << m_logPrefix << "unable to find remote_sensor_boards into config file.";
        return false;
    }

    // open the iAnalogsensor YARP device
    yarp::os::Property optionsAnalogDevice;

    optionsAnalogDevice.put("device", "analogsensorclient");
    optionsAnalogDevice.put("local", "/" + robot + "/" + iCubSensorPart + "/analog:i");
    optionsAnalogDevice.put("remote", "/" + robot + "/" + iCubSensorPart + "/analog:o");

    if (!m_analogDevice.open(optionsAnalogDevice))
    {
        yError() << m_logPrefix << "could not open analogSensorClient object.";
        return false;
    }

    if (!m_analogDevice.view(m_analogSensorInterface) || !m_analogSensorInterface)
    {
        yError() << m_logPrefix << "cannot obtain IAnalogSensor interface";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property optionsRobotDevice;
    optionsRobotDevice.put("device", "remotecontrolboardremapper");

    YarpHelper::addVectorOfStringToProperty(optionsRobotDevice, "axesNames", m_actuatedAxisNames);

    // prepare the remotecontrolboards
    yarp::os::Bottle remoteControlBoards;
    remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    optionsRobotDevice.put("remoteControlBoards", remoteControlBoards.get(0));
    optionsRobotDevice.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts
        = optionsRobotDevice.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    bool useVelocity = config.check("useVelocity", yarp::os::Value(false)).asBool();
    m_controlMode = useVelocity ? VOCAB_CM_VELOCITY : VOCAB_CM_POSITION_DIRECT;
    m_pidControlMode
        = useVelocity ? yarp::dev::VOCAB_PIDTYPE_VELOCITY : yarp::dev::VOCAB_PIDTYPE_POSITION;

    // open the device
    if (!m_robotDevice.open(optionsRobotDevice) && m_isMandatory)
    {
        yError() << m_logPrefix << "could not open remotecontrolboardremapper object.";
        return false;
    }

    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << m_logPrefix << "cannot obtain IEncoders interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain "
                    "IPositionControl interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << m_logPrefix << "cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << m_logPrefix << "cannot obtain IVelocityInterface interface";
        return false;
    }

    if (!m_robotDevice.view(m_limitsInterface) || !m_limitsInterface)
    {
        yError() << m_logPrefix << "cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IControlMode "
                    "interface";
        return false;
    }

    if (!m_robotDevice.view(m_timedInterface) || !m_timedInterface)
    {
        yError() << m_logPrefix << "cannot obtain iTimed interface";
        return false;
    }

    if (!m_robotDevice.view(m_currentInterface) || !m_currentInterface)
    {
        yError() << m_logPrefix << "cannot obtain ICurrentControl interface";
        return false;
    }

    if (!m_robotDevice.view(m_pwmInterface) || !m_pwmInterface)
    {
        yError() << m_logPrefix << "cannot obtain IPWMControl interface";
        return false;
    }

    if (!m_robotDevice.view(m_pidInterface) || !m_pidInterface)
    {
        yError() << m_logPrefix << "cannot obtain IPidControl interface";
        return false;
    }

    // set the reference velocity for the position control mode
    m_refenceVelocityForPositionControl
        = config.check("referenceVelocityForPositionControl", yarp::os::Value(10.0)).asDouble();

    yarp::sig::Vector dummy(m_noActuatedAxis, m_refenceVelocityForPositionControl);

    if (!m_positionInterface->setRefSpeeds(dummy.data()) && m_isMandatory)
    {
        yError() << m_logPrefix
                 << "Error while setting the desired velocity to the position interface.";
        return false;
    }

    // resize the vectors

    // feedbacks
    m_encoderPositionFeedbackInDegrees.resize(m_noActuatedAxis);
    m_encoderPositionFeedbackInRadians.resize(m_noActuatedAxis);
    m_encoderVelocityFeedbackInDegrees.resize(m_noActuatedAxis);
    m_encoderVelocityFeedbackInRadians.resize(m_noActuatedAxis);

    m_actuatedJointFeedbacksInRadian.resize(m_noActuatedJoints);

    m_analogSensorFeedbackRaw.resize(m_noAnalogSensor);
    m_analogSensorFeedbackInDegrees.resize(m_noAnalogSensor);
    m_analogSensorFeedbackInRadians.resize(m_noAnalogSensor);

    m_motorCurrentFeedbacks.resize(m_noActuatedAxis);
    m_motorPwmFeedbacks.resize(m_noActuatedAxis);
    m_pidOutput.resize(m_noActuatedAxis);

    // reference
    m_referenceValues.resize(m_noActuatedAxis);
    m_axisPositionReferences.resize(m_noActuatedAxis);
    m_axisPositionDirectReferences.resize(m_noActuatedAxis);
    m_axisVelocityReferences.resize(m_noActuatedAxis);
    m_motorCurrentReferences.resize(m_noActuatedAxis);
    m_motorPwmReferences.resize(m_noActuatedAxis);

    // check if the robot is alive
    bool okPosition = false;
    for (int i = 0; i < 10 && !okPosition; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_encoderPositionFeedbackInDegrees.data());

        if (!okPosition)
            yarp::os::Time::delay(0.1);
    }
    if (!okPosition)
    {
        yError() << m_logPrefix << "unable to read encoders (position).";
        return false;
    }

    double intializationTime
        = config.check("robotInitializationTime", yarp::os::Value(5.0)).asDouble();

    m_steadyStateCounterThreshold
        = config.check("steadyStateCounterThreshold", yarp::os::Value(5)).asInt64();
    m_steadyStateThreshold = config.check("steadyStateThreshold", yarp::os::Value(0.05)).asDouble();
    m_steadyStateCounter = 0;

    if (!initializeAxisValues(intializationTime))
    {
        yError() << m_logPrefix << "unable to initialize the axis values to their minimum values.";
        return false;
    }

    if (!switchToControlMode(m_controlMode))
    {
        yError() << m_logPrefix << "Unable to switch the control mode";
        return false;
    }
    // print information:
    yInfo() << m_logPrefix << "actuated axis:" << m_actuatedAxisNames;
    yInfo() << m_logPrefix << "actuated joints:" << m_actuatedJointList;
    for (int i = 0; i < m_noActuatedJoints; i++)
    {
        const std::string& jointName = m_actuatedJointList[i];
        const auto& element = m_jointInfoMap[jointName];
        yInfo() << m_logPrefix << "actuated joint info" << jointName
                << " ,use analog:" << element.useAnalog << " ,index: " << element.index
                << " ,scale:" << element.scale;
    }
    yInfo() << m_logPrefix << "m_noAllAxis: " << m_noAllAxis;
    yInfo() << m_logPrefix << "m_noAllJoints: " << m_noAllJoints;
    yInfo() << m_logPrefix << "m_allAxisNames: " << m_allAxisNames;
    yInfo() << m_logPrefix << "m_allJointNames: " << m_allJointNames;

    return true;
}

bool RobotInterface::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if (!m_controlModeInterface)
    {
        yError() << m_logPrefix << "ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_noActuatedAxis, controlMode);
    if (!m_controlModeInterface->setControlModes(controlModes.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Error while setting the controlMode.";
        return false;
    }

    return true;
}

bool RobotInterface::initializeAxisValues(const double& initializationTime)
{
    std::vector<double> minLimits, maxLimits;

    yInfo() << m_logPrefix << "the necessary time for initialization is: " << initializationTime
            << " seconds.";
    if (!this->getActuatedAxisLimits(minLimits, maxLimits))
    {
        yError() << m_logPrefix << "unable to get the robot axis limits.";
        return false;
    }

    if (!switchToControlMode(VOCAB_CM_POSITION))
    {
        yError() << m_logPrefix << "Unable to switch in position control.";
        return false;
    }

    if (!this->setAxisReferences(minLimits, VOCAB_CM_POSITION))
    {
        yInfo() << m_logPrefix << "unable to set axis references.";
        return false;
    }

    bool steadyStateReached = false;
    double startingTime = yarp::os::Time::now();
    std::vector<double> feedbacks;
    do
    {
        this->getFeedback();
        this->axisFeedbacks(feedbacks);
        steadyStateReached = this->isSteadyStateReached(minLimits, feedbacks);
        yarp::os::Time::delay(0.01); // wait for 0.01 seconds to reach the min values

        if ((yarp::os::Time::now() - startingTime) > initializationTime)
        {
            yError() << m_logPrefix << "unable to initialize the robot axis values";
            return false;
        }

    } while (!steadyStateReached);

    return true;
}

bool RobotInterface::setDirectPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionDirectInterface == nullptr)
    {
        yError() << m_logPrefix << "PositionDirect I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_noActuatedAxis)
    {
        yError() << m_logPrefix
                 << "dimension mismatch between desired position vector and the number of "
                    "controlled joints.";
        return false;
    }

    m_axisPositionDirectReferences = desiredPosition;

    // convert radiant to degree
    for (int i = 0; i < m_noActuatedAxis; i++)
        m_referenceValues(i) = iDynTree::rad2deg(desiredPosition(i));

    // set desired position
    if (!m_positionDirectInterface->setPositions(m_referenceValues.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotInterface::setPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionInterface == nullptr)
    {
        yError() << m_logPrefix << "Position I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_noActuatedAxis)
    {
        yError() << m_logPrefix
                 << "Dimension mismatch between desired position vector and the number of "
                    "controlled joints.";
        return false;
    }

    m_axisPositionReferences = desiredPosition;

    // convert radiant to degree
    for (int i = 0; i < m_noActuatedAxis; i++)
        m_referenceValues(i) = iDynTree::rad2deg(desiredPosition(i));

    // set desired position
    if (!m_positionInterface->positionMove(m_referenceValues.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotInterface::setVelocityReferences(const yarp::sig::Vector& desiredVelocity)
{
    if (m_velocityInterface == nullptr)
    {
        yError() << m_logPrefix << "Velocity I/F not ready.";
        return false;
    }

    if (desiredVelocity.size() != m_noActuatedAxis)
    {
        yError() << m_logPrefix
                 << "Dimension mismatch between desired velocity vector and the number of "
                    "controlled joints.";
        return false;
    }

    m_axisVelocityReferences = desiredVelocity;

    // convert radiant/s  to degree/s
    for (int i = 0; i < m_noActuatedAxis; i++)
        m_referenceValues(i) = iDynTree::rad2deg(desiredVelocity(i));

    // since the velocity interface use a minimum jerk trajectory a very high
    // acceleration is set in order to use it as velocity "direct" interface
    yarp::sig::Vector dummy(m_noActuatedAxis, std::numeric_limits<double>::max());
    if (!m_velocityInterface->setRefAccelerations(dummy.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << " Error while setting the desired acceleration.";
        return false;
    }

    if (!m_velocityInterface->velocityMove(m_referenceValues.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Error while setting the desired velocity.";
        return false;
    }
    return true;
}

bool RobotInterface::setCurrentReferences(const yarp::sig::Vector& desiredCurrent)
{
    if (m_currentInterface == nullptr)
    {
        yError() << m_logPrefix << "Current I/F not ready.";
        return false;
    }

    if (desiredCurrent.size() != m_noActuatedAxis)
    {
        yError() << m_logPrefix
                 << "Dimension mismatch between desired current vector and the "
                    "number of controlled joints.";
        return false;
    }

    // update the desired current values
    m_motorCurrentReferences = desiredCurrent;
    m_referenceValues = desiredCurrent;

    // set desired current
    if (!m_currentInterface->setRefCurrents(m_referenceValues.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Error while setting the desired current.";
        return false;
    }
    return true;
}

bool RobotInterface::setPwmReferences(const yarp::sig::Vector& desiredPwm)
{
    if (m_pwmInterface == nullptr)
    {
        yError() << m_logPrefix << "PWM I/F not ready.";
        return false;
    }

    if (desiredPwm.size() != m_noActuatedAxis)
    {
        yError() << m_logPrefix
                 << "Dimension mismatch between desired current vector and the number of "
                    "controlled joints.";
        return false;
    }

    // update the desired PWM values
    m_motorPwmReferences = desiredPwm;
    m_referenceValues = desiredPwm;

    // set desired PWM
    if (!m_pwmInterface->setRefDutyCycles(m_referenceValues.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Error while setting the desired PWM.";
        return false;
    }
    return true;
}

void RobotInterface::updateTimeStamp()
{
    if (m_timedInterface)
        m_timeStamp = m_timedInterface->getLastInputStamp();
    else
        m_timeStamp.update();
}

bool RobotInterface::getFeedback()
{
    double time0 = yarp::os::Time::now();

    if (!m_encodersInterface->getEncoders(m_encoderPositionFeedbackInDegrees.data())
        && m_isMandatory)
    {
        yError() << m_logPrefix << "Unable to get axes position.";
        return false;
    }

    for (unsigned j = 0; j < m_noActuatedAxis; ++j)
        m_encoderPositionFeedbackInRadians(j)
            = iDynTree::deg2rad(m_encoderPositionFeedbackInDegrees(j));

    if (!m_encodersInterface->getEncoderSpeeds(m_encoderVelocityFeedbackInDegrees.data())
        && m_isMandatory)
    {
        yError() << m_logPrefix << "Unable to get axes velocity feedback.";
        return false;
    }

    for (unsigned j = 0; j < m_noActuatedAxis; ++j)
        m_encoderVelocityFeedbackInRadians(j)
            = iDynTree::deg2rad(m_encoderVelocityFeedbackInDegrees(j));

    if (!(m_analogSensorInterface->read(m_analogSensorFeedbackRaw)
          == yarp::dev::IAnalogSensor::AS_OK))
    {
        yError() << m_logPrefix << "Unable to get analog sensor data.";
        return false;
    }

    if (!computeCalibratedAnalogSesnors())
    {
        yError() << m_logPrefix << "Unable to get the calibrated analog sensor data.";
        return false;
    }

    if (!computeActuatedJointFeedbacks())
    {
        yError() << m_logPrefix << "Unable to set all the actuated joints feedback sensor data.";
        return false;
    }

    if (!m_currentInterface->getCurrents(m_motorCurrentFeedbacks.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Unable to get motor current feedbacks.";
        return false;
    }

    if (!m_pwmInterface->getDutyCycles(m_motorPwmFeedbacks.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Unable to get motor PWM feedbacks.";
        return false;
    }

    if (!m_pidInterface->getPidOutputs(m_pidControlMode, m_pidOutput.data()) && m_isMandatory)
    {
        yError() << m_logPrefix << "Unable to get pid outputs.";
        return false;
    }
    return true;
}

bool RobotInterface::computeCalibratedAnalogSesnors()
{
    for (unsigned j = 0; j < m_noAnalogSensor; ++j)
    {
        m_analogSensorFeedbackInDegrees(j)
            = m_analogJointsMinBoundaryDegree(j)
              + m_sensorsRaw2DegreeScaling(j)
                    * (m_analogSensorFeedbackRaw(j) - m_analogSensorsRawMinBoundary(j));

        m_analogSensorFeedbackInRadians(j) = iDynTree::deg2rad(m_analogSensorFeedbackInDegrees(j));
    }

    return true;
}

bool RobotInterface::computeActuatedJointFeedbacks() // update this function later
{

    for (size_t idx = 0; idx < m_noActuatedJoints; idx++)
    {
        const std::string& jointName = m_actuatedJointList[idx];
        const auto& element = m_jointInfoMap[jointName];
        //        yInfo() << m_logPrefix << "index: " << idx << " , joint name: " << jointName
        //                << " , use analog: " << element.useAnalog
        //                << ", analog/axis index: " << element.index << ", scale: " <<
        //                element.scale;

        if (element.useAnalog)
        {
            m_actuatedJointFeedbacksInRadian(idx) = m_analogSensorFeedbackInRadians(element.index);
        } else
        {
            m_actuatedJointFeedbacksInRadian(idx)
                = m_encoderPositionFeedbackInRadians(element.index) * element.scale;
        }
    }

    return true;
}

const yarp::os::Stamp& RobotInterface::timeStamp() const
{
    return m_timeStamp;
}

const yarp::sig::Vector& RobotInterface::axisFeedbacks() const
{
    return m_encoderPositionFeedbackInRadians;
}

void RobotInterface::axisFeedbacks(std::vector<double>& axisFeedbacks)
{
    CtrlHelper::toStdVector(m_encoderPositionFeedbackInRadians, axisFeedbacks);
}

const yarp::sig::Vector& RobotInterface::axisPositionReferences() const
{
    return m_axisPositionReferences;
}

void RobotInterface::axisPositionReferences(std::vector<double>& axisPositionReferences)
{
    CtrlHelper::toStdVector(m_axisPositionReferences, axisPositionReferences);
}

const yarp::sig::Vector& RobotInterface::axisPositionDirectReferences() const
{
    return m_axisPositionDirectReferences;
}

void RobotInterface::axisPositionDirectReferences(std::vector<double>& axisPositionDirectReferences)
{
    CtrlHelper::toStdVector(m_axisPositionDirectReferences, axisPositionDirectReferences);
}

const yarp::sig::Vector& RobotInterface::axisVelocityFeedbacks() const
{
    return m_encoderVelocityFeedbackInRadians;
}

void RobotInterface::axisVelocityFeedbacks(std::vector<double>& axisVelocityFeedbacks)
{
    CtrlHelper::toStdVector(m_encoderVelocityFeedbackInRadians, axisVelocityFeedbacks);
}

const yarp::sig::Vector& RobotInterface::axisVelocityReferences() const
{
    return m_axisVelocityReferences;
}

void RobotInterface::axisVelocityReferences(std::vector<double>& axisVelocityReferences)
{
    CtrlHelper::toStdVector(m_axisVelocityReferences, axisVelocityReferences);
}

const yarp::sig::Vector& RobotInterface::analogSensorFeedbacks() const
{
    return m_analogSensorFeedbackInRadians;
}

const yarp::sig::Vector& RobotInterface::actuatedJointFeedbacks() const
{
    return m_actuatedJointFeedbacksInRadian;
}

void RobotInterface::actuatedJointFeedbacks(std::vector<double>& jointsFeedbacks)
{
    CtrlHelper::toStdVector(m_actuatedJointFeedbacksInRadian, jointsFeedbacks);
}

const yarp::sig::Vector& RobotInterface::motorCurrents() const
{
    return m_motorCurrentFeedbacks;
}

void RobotInterface::motorCurrents(std::vector<double>& motorCurrents)
{
    CtrlHelper::toStdVector(m_motorCurrentFeedbacks, motorCurrents);
}

const yarp::sig::Vector& RobotInterface::motorCurrentReference() const
{
    return m_motorCurrentReferences;
}

void RobotInterface::motorCurrentReference(std::vector<double>& motorCurrentReferences)
{
    CtrlHelper::toStdVector(m_motorCurrentReferences, motorCurrentReferences);
}

const yarp::sig::Vector& RobotInterface::motorPwm() const
{
    return m_motorPwmFeedbacks;
}

void RobotInterface::motorPwm(std::vector<double>& motorPwm)
{
    CtrlHelper::toStdVector(m_motorPwmFeedbacks, motorPwm);
}

const yarp::sig::Vector& RobotInterface::motorPwmReference() const
{
    return m_motorPwmReferences;
}

void RobotInterface::motorPwmReference(std::vector<double>& motorPwmReference)
{
    CtrlHelper::toStdVector(m_motorPwmReferences, motorPwmReference);
}

const yarp::sig::Vector& RobotInterface::motorPidOutputs() const
{
    return m_pidOutput;
}

void RobotInterface::motorPidOutputs(std::vector<double>& motorPidOutputs)
{
    CtrlHelper::toStdVector(m_pidOutput, motorPidOutputs);
}

bool RobotInterface::close()
{
    yInfo() << m_logPrefix << "closing.";
    if (!switchToControlMode(VOCAB_CM_POSITION))
        yError() << m_logPrefix << "Unable to switch in position control.";
    return false;

    if (!m_robotDevice.close())
    {
        yError() << m_logPrefix << "Unable to close the remotecontrolboardremepper device.";
        return false;
    }

    if (!m_analogDevice.close())
    {
        yError() << m_logPrefix << "Unable to close the analogsensorclient device.";
        return false;
    }
    m_timedInterface = nullptr;
    m_encodersInterface = nullptr;
    m_positionDirectInterface = nullptr;
    m_positionInterface = nullptr;
    m_velocityInterface = nullptr;
    m_controlModeInterface = nullptr;
    m_limitsInterface = nullptr;
    m_analogSensorInterface = nullptr;
    m_currentInterface = nullptr;
    m_pwmInterface = nullptr;
    m_pidInterface = nullptr;

    yInfo() << m_logPrefix << "closed correctly.";
    return true;
}

const int RobotInterface::getNumberOfActuatedAxis() const
{
    return m_noActuatedAxis;
}

const int RobotInterface::getNumberOfAllAxis() const
{
    return m_noAllAxis;
}

const int RobotInterface::getNumberOfAllJoints() const
{
    return m_noAllJoints;
}

const int RobotInterface::getNumberOfActuatedJoints() const
{
    return m_noActuatedJoints;
}

void RobotInterface::getActuatedJointNames(std::vector<std::string>& names) const
{
    names = m_actuatedJointList;
}
void RobotInterface::getAllJointNames(std::vector<std::string>& names) const
{
    names = m_allJointNames;
}

void RobotInterface::getActuatedAxisNames(std::vector<std::string>& names) const
{
    names = m_actuatedAxisNames;
}

void RobotInterface::getAllAxisNames(std::vector<std::string>& names) const
{
    names = m_allAxisNames;
}

bool RobotInterface::getActuatedAxisLimits(yarp::sig::Matrix& limits)
{
    if (!getFeedback())
    {
        yError() << m_logPrefix << "Unable to get the feedback from the robot";
        return false;
    }

    // resize matrix
    limits.resize(m_noActuatedAxis, 2);

    double maxLimitInDegree, minLimitInDegree;
    for (int i = 0; i < m_noActuatedAxis; i++)
    {
        // get position limits
        if (!m_limitsInterface->getLimits(i, &minLimitInDegree, &maxLimitInDegree))
        {
            if (m_isMandatory)
            {
                yError() << m_logPrefix << "Unable get " << m_actuatedAxisNames[i]
                         << " joint limits.";
                return false;
            } else
            {
                limits(i, 0) = m_encoderPositionFeedbackInRadians(i);
                limits(i, 1) = m_encoderPositionFeedbackInRadians(i);
                yWarning()
                    << m_logPrefix << "Unable get " << m_actuatedAxisNames[i]
                    << " joint limits. The current joint value is used as lower and upper limits.";
            }
        } else
        {
            limits(i, 0) = iDynTree::deg2rad(minLimitInDegree);
            limits(i, 1) = iDynTree::deg2rad(maxLimitInDegree);
        }
    }
    return true;
}

bool RobotInterface::getActuatedAxisLimits(yarp::sig::Vector& minLimits,
                                           yarp::sig::Vector& maxLimits)
{
    std::vector<double> minValues, maxValues;
    if (!this->getActuatedAxisLimits(minValues, maxValues))
    {
        yError() << m_logPrefix << "unable to get the robot axis limits";
        return false;
    }

    CtrlHelper::toYarpVector(minValues, minLimits);
    CtrlHelper::toYarpVector(maxValues, maxLimits);

    return true;
}

bool RobotInterface::getActuatedAxisLimits(std::vector<double>& minLimits,
                                           std::vector<double>& maxLimits)
{
    minLimits.resize(m_noActuatedAxis, 0.0);
    maxLimits.resize(m_noActuatedAxis, 0.0);

    yarp::sig::Matrix limits;
    if (!this->getActuatedAxisLimits(limits))
    {
        yInfo() << m_logPrefix << "unable to get the robot axis limits";
        return false;
    }

    for (size_t i = 0; i < m_noActuatedAxis; i++)
    {
        minLimits[i] = limits(i, 0);
        maxLimits[i] = limits(i, 1);
    }
    return true;
}

bool RobotInterface::getActuatedJointLimits(std::vector<double>& minLimits,
                                            std::vector<double>& maxLimits)
{
    minLimits.resize(m_noActuatedJoints, 0.0);
    maxLimits.resize(m_noActuatedJoints, 0.0);

    // get the axis limits
    std::vector<double> axisMinLimits, axisMaxLimits;
    this->getActuatedAxisLimits(axisMinLimits, axisMaxLimits);

    // compute the joint limits
    for (size_t idx = 0; idx < m_noActuatedJoints; idx++)
    {
        const std::string& jointName = m_actuatedJointList[idx];
        const auto& element = m_jointInfoMap[jointName];
        if (element.useAnalog)
        {
            minLimits[idx] = iDynTree::deg2rad(m_analogJointsMinBoundaryDegree(element.index));
            maxLimits[idx] = iDynTree::deg2rad(m_analogJointsMaxBoundaryDegree(element.index));
        } else
        {
            minLimits[idx] = axisMinLimits[element.index] * element.scale;
            maxLimits[idx] = axisMaxLimits[element.index] * element.scale;
        }
    }
    return true;
}

bool RobotInterface::getVelocityLimits(yarp::sig::Matrix& limits)
{
    if (!getFeedback())
    {
        yError() << m_logPrefix << "Unable to get the feedback from the robot";
        return false;
    }
    // resize matrix
    limits.resize(m_noActuatedAxis, 2);

    double maxLimitInDegree, minLimitInDegree;
    for (int i = 0; i < m_noActuatedAxis; i++)
    {
        // get position limits
        if (!m_limitsInterface->getVelLimits(i, &minLimitInDegree, &maxLimitInDegree))
        {
            yError() << m_logPrefix << "Unable get " << m_actuatedAxisNames[i] << " joint limits.";
            return false;

        } else
        {
            limits(i, 0) = iDynTree::deg2rad(minLimitInDegree);
            limits(i, 1) = iDynTree::deg2rad(maxLimitInDegree);
        }
    }
    return true;
}

bool RobotInterface::setAxisReferences(std::vector<double>& desiredValues)
{
    return this->setAxisReferences(desiredValues, m_controlMode);
}

bool RobotInterface::setAxisReferences(std::vector<double>& desiredValues, const int& controlMode)
{
    if (desiredValues.size() != m_noActuatedAxis)
    {
        yError() << m_logPrefix
                 << "the number of input data is not equal to the actuated number of robot axis.";
        return false;
    }
    yarp::sig::Vector desiredValue(m_noActuatedAxis, 0.0);
    CtrlHelper::toYarpVector(desiredValues, desiredValue);

    switch (controlMode)
    {
    case VOCAB_CM_POSITION_DIRECT:
        if (!setDirectPositionReferences(desiredValue))
        {
            yError() << m_logPrefix << "Unable to set the axis position-direct  references.";
            return false;
        }
        break;

    case VOCAB_CM_POSITION:
        if (!setPositionReferences(desiredValue))
        {
            yError() << m_logPrefix << "Unable to set the axis position references.";
            return false;
        }
        break;

    case VOCAB_CM_VELOCITY:
        if (!setVelocityReferences(desiredValue))
        {
            yError() << m_logPrefix << "Unable to set the axis velocity references.";
            return false;
        }
        break;

    case VOCAB_CM_CURRENT:
        if (!setCurrentReferences(desiredValue))
        {
            yError() << m_logPrefix << "Unable to set the motor current references.";
            return false;
        }
        break;

    case VOCAB_CM_PWM:
        if (!setPwmReferences(desiredValue))
        {
            yError() << m_logPrefix << "Unable to set the motor PWM references.";
            return false;
        }
        break;

    default:
        yError() << m_logPrefix << "Unknown control mode.";
        return false;
    }

    return true;
}

bool RobotInterface::isVelocityControlUsed()
{
    return m_controlMode == VOCAB_CM_VELOCITY;
}

bool RobotInterface::isSteadyStateReached(yarp::sig::Vector& reference, yarp::sig::Vector& feedback)
{
    std::vector<double> referenceStd;
    std::vector<double> feedbackStd;
    CtrlHelper::toStdVector(reference, referenceStd);
    CtrlHelper::toStdVector(feedback, feedbackStd);

    return this->isSteadyStateReached(referenceStd, feedbackStd);
}

bool RobotInterface::isSteadyStateReached(std::vector<double>& reference,
                                          std::vector<double>& feedback)
{
    bool tmp = true;
    if (reference.size() != feedback.size())
    {
        yError() << m_logPrefix << "the reference and feedback vector sizes are not equal.";
        return false;
    }
    for (size_t i = 0; i < reference.size(); i++)
    {
        tmp &= std::abs(reference[i] - feedback[i]) <= m_steadyStateThreshold;
    }

    if (tmp)
        m_steadyStateCounter++;
    else
        m_steadyStateCounter = 0;

    bool steadyStateReached = false;

    if (m_steadyStateCounter >= m_steadyStateCounterThreshold)
        steadyStateReached = true;

    return steadyStateReached;
}
