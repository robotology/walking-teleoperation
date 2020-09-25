/**
 * @file HapticGloveRobotControlHelper.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <limits>

// iDynTree
#include <iDynTree/Core/Utils.h>

#include <HapticGloveRobotControlHelper.hpp>
#include <Utils.hpp>

using namespace HapticGlove;

bool RobotControlHelper::configure(const yarp::os::Searchable& config,
                                   const std::string& name,
                                   bool isMandatory)
{
    m_isMandatory = isMandatory;

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value* iCubPartsYarp;
    if (!config.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find remote_control_boards into "
                    "config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a vector of "
                    "strings.";
        return false;
    }

    // get all icub senosry parts from the resource finder
    std::string iCubSensorPart;

    if (!YarpHelper::getStringFromSearchable(config, "remote_sensor_boards", iCubSensorPart))
    {
        yError() << "[RobotControlHelper::configure] Unable to find remote_sensor_boards into "
                    "config file.";
        return false;
    }
    // open the iAnalogsensor YARP device
    yarp::os::Property optionsAnalogDevice;
    optionsAnalogDevice.put("device", "analogsensorclient");
    optionsAnalogDevice.put("local", "/" + robot + "/" + iCubSensorPart + "/analog:i");
    optionsAnalogDevice.put("remote", "/" + robot + "/" + iCubSensorPart + "/analog:o");

    if (!m_analogDevice.open(optionsAnalogDevice))
    {
        yError() << "[RobotControlHelper::configure] Could not open analogsensorclient "
                    "object.";
        return false;
    }

    if (!m_analogDevice.view(m_AnalogSensorInterface) || !m_AnalogSensorInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IAnalogSensor interface";
        return false;
    }

    m_noAnalogSensor = config.check("noAnalogSensor", yarp::os::Value(15)).asInt();
    yInfo() << "m_noAnalogSensor " << m_noAnalogSensor;

    m_noAllSensor = config.check("noAllSensor", yarp::os::Value(17)).asInt(); // 5*3 analog sensors+ 1 encoder thumb oppose + 1 encoder hand_finger
    yInfo() << "m_noAllSensor " << m_noAllSensor;


    m_analogSensorFeedbackRaw.resize(15); //ToFix
    m_analogSensorFeedbackInDegrees.resize(m_noAnalogSensor);
    m_analogSensorFeedbackInRadians.resize(m_noAnalogSensor);
    m_analogSensorFeedbackSelected = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13};

    m_allSensorFeedbackInRadians.resize(m_noAllSensor);

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property optionsRobotDevice;
    yarp::os::Value* axesListYarp;
    if (!config.check("axis_list", axesListYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find joints_list into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }


    m_joints_min_boundary.resize(m_noAnalogSensor, 0.0); 
    m_joints_max_boundary.resize(m_noAnalogSensor, 0.0);  
    m_sensors_min_boundary.resize(m_noAnalogSensor, 0.0); 
    m_sensors_max_boundary.resize(m_noAnalogSensor, 0.0); 
    m_sensors_raw2Degree_scaling.resize(m_noAnalogSensor, 0.0); 

        // get the joints limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "joints_min_boundary", m_joints_min_boundary))
    {
        yError() << "RobotControlHelper::configure] unable to get the minimum boundary of the joints limits.";
        return false;
    }

     if (!YarpHelper::getYarpVectorFromSearchable(
            config, "joints_max_boundary", m_joints_max_boundary))
    {
        yError() << "RobotControlHelper::configure] unable to get the maximum boundary of the "
                    "joints limits.";
        return false;
    }

        // get the sensors limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "sensors_min_boundary", m_sensors_min_boundary))
    {
        yError() << "RobotControlHelper::configure] unable to get the minimum boundary of the "
                    "joints limits.";
        return false;
    }

            // get the sensors limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "sensors_max_boundary", m_sensors_max_boundary))
    {
        yError() << "RobotControlHelper::configure] unable to get the maximum boundary of the "
                    "joints limits.";
        return false;
    }

    for (size_t i = 0; i < m_noAnalogSensor; i++)
    {
        m_sensors_raw2Degree_scaling(i)
            = double(m_joints_max_boundary(i) - m_joints_min_boundary(i))
              / double(m_sensors_max_boundary(i)- m_sensors_min_boundary(i));
    }

    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "sensors_max_boundary", m_joints_max_boundary))
    {
        yError() << "RobotControlHelper::configure] unable to get the maximum boundary of the "
                    "joints limits.";
        return false;
    }

    optionsRobotDevice.put("device", "remotecontrolboardremapper");

    YarpHelper::addVectorOfStringToProperty(optionsRobotDevice, "axesNames", m_axesList);

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

    m_actuatedDOFs = m_axesList.size();

    bool useVelocity = config.check("useVelocity", yarp::os::Value(false)).asBool();
    m_controlMode = useVelocity ? VOCAB_CM_VELOCITY : VOCAB_CM_POSITION_DIRECT;

    // open the device
    if (!m_robotDevice.open(optionsRobotDevice) && m_isMandatory)
    {
        yError() << "[RobotControlHelper::configure] Could not open remotecontrolboardremapper "
                    "object.";
        return false;
    }

    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IEncoders interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionControl interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IVelocityInterface interface";
        return false;
    }

    if (!m_robotDevice.view(m_limitsInterface) || !m_limitsInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IControlMode interface";
        return false;
    }

    if (!m_robotDevice.view(m_timedInterface) || !m_timedInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain iTimed interface";
        return false;
    }

    m_desiredJointValue.resize(m_actuatedDOFs);
    m_encoderPositionFeedbackInDegrees.resize(m_actuatedDOFs);
    m_encoderPositionFeedbackInRadians.resize(m_actuatedDOFs);

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
        yError() << "[RobotControlHelper::configure] Unable to read encoders (position).";
        return false;
    }

    if (!switchToControlMode(m_controlMode))
    {
        yError() << "[RobotControlHelper::configure] Unable to switch the control mode";
        return false;
    }
    return true;
}

bool RobotControlHelper::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if (!m_controlModeInterface)
    {
        yError() << "[RobotControlHelper::switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if (!m_controlModeInterface->setControlModes(controlModes.data()) && m_isMandatory)
    {
        yError()
            << "[RobotControlHelper::switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool RobotControlHelper::setDirectPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionDirectInterface == nullptr)
    {
        yError()
            << "[RobotControlHelper::setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlHelper::setDirectPositionReferences] Dimension mismatch between "
                    "desired position vector and the number of controlled joints.";
        return false;
    }

    // convert radiant to degree
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredPosition(i));

    // set desired position
    if (!m_positionDirectInterface->setPositions(m_desiredJointValue.data()) && m_isMandatory)
    {
        yError() << "[RobotControlHelper::setDirectPositionReferences] Error while setting the "
                    "desired position.";
        return false;
    }

    return true;
}

bool RobotControlHelper::setPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionInterface == nullptr)
    {
        yError() << "[RobotControlHelper::setPositionReferences] Position I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlHelper::setDirectPositionReferences] Dimension mismatch between "
                    "desired position vector and the number of controlled joints.";
        return false;
    }

    // convert radiant to degree
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredPosition(i));

    // set desired position
    if (!m_positionInterface->positionMove(m_desiredJointValue.data()) && m_isMandatory)
    {
        yError() << "[RobotControlHelper::setPositionReferences] Error while setting the "
                    "desired position.";
        return false;
    }

    return true;
}

bool RobotControlHelper::setVelocityReferences(const yarp::sig::Vector& desiredVelocity)
{
    if (m_velocityInterface == nullptr)
    {
        yError() << "[RobotControlHelper::setVelocityReferences] Velocity I/F not ready.";
        return false;
    }

    if (desiredVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlHelper::setVelocityReferences] Dimension mismatch between "
                    "desired velocity vector and the number of controlled joints.";
        return false;
    }

    // convert radiant/s  to degree/s
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredVelocity(i));

    // since the velocity interface use a minimum jerk trajectory a very high acceleration is set in
    // order to use it as velocity "direct" interface
    yarp::sig::Vector dummy(m_actuatedDOFs, std::numeric_limits<double>::max());
    if (!m_velocityInterface->setRefAccelerations(dummy.data()) && m_isMandatory)
    {
        yError() << "[RobotControlHelper::setVelocityReferences] Error while setting the desired "
                    "acceleration.";
        return false;
    }

    if (!m_velocityInterface->velocityMove(m_desiredJointValue.data()) && m_isMandatory)
    {
        yError() << "[RobotControlHelper::setVelocityReferences] Error while setting the desired "
                    "velocity.";
        return false;
    }
    return true;
}

void RobotControlHelper::updateTimeStamp()
{
    if (m_timedInterface)
        m_timeStamp = m_timedInterface->getLastInputStamp();
    else
        m_timeStamp.update();
}

bool RobotControlHelper::getFeedback()
{
    if (!m_encodersInterface->getEncoders(m_encoderPositionFeedbackInDegrees.data()) && m_isMandatory)
    {
        yError() << "[RobotControlHelper::getFeedbacks] Unable to get joint position";
        return false;
    }

    for (unsigned j = 0; j < m_actuatedDOFs; ++j)
        m_encoderPositionFeedbackInRadians(j) = iDynTree::deg2rad(m_encoderPositionFeedbackInDegrees(j));
    yInfo() << "m_encoderPositionFeedbackInDegrees" << m_encoderPositionFeedbackInDegrees.toString();
    yInfo() << "m_encoderPositionFeedbackInRadians" << m_encoderPositionFeedbackInRadians.toString();


    if (!(m_AnalogSensorInterface->read(m_analogSensorFeedbackRaw) == yarp::dev::IAnalogSensor::AS_OK))
    {
        yError() << "[RobotControlHelper::getFeedbacks] Unable to get analog sensor data";
        return false;
    }
    if (!getCalibratedFeedback())
    {
        yError() << "[RobotControlHelper::getFeedbacks] Unable to get the calibrated analog "
                    "sensor data";
        return false;
    }

    if(!setAllJointsFeedback())
    {
        yError() << "[RobotControlHelper::getFeedbacks] Unable to set all the interested joints feedback"
                    "sensor data";
        return false;
    }

    return true;
}

bool RobotControlHelper::getCalibratedFeedback()
{

    for (unsigned j = 0; j < m_noAnalogSensor; ++j)
    {
        m_analogSensorFeedbackInDegrees(j) = m_joints_min_boundary(j)
                                       + m_sensors_raw2Degree_scaling(j)
                                             * (m_analogSensorFeedbackRaw(m_analogSensorFeedbackSelected(j))
                                                - m_sensors_min_boundary(j)); // TOCHECK
        m_analogSensorFeedbackInRadians(j) = iDynTree::deg2rad(m_analogSensorFeedbackInDegrees(j));
    }
    yInfo() << "m_sensorFeedbackInDegrees" << m_analogSensorFeedbackInDegrees.toString();
    yInfo() << "m_sensorFeedbackInRadians" << m_analogSensorFeedbackInRadians.toString();


    return true;
}

bool RobotControlHelper::setAllJointsFeedback()
{


    m_allSensorFeedbackInRadians(0)=m_encoderPositionFeedbackInRadians(0);
    for(unsigned j = 0; j < m_noAnalogSensor; ++j)
    {
        m_allSensorFeedbackInRadians(j+1)=m_analogSensorFeedbackInRadians(j);
    }

    yInfo() << "m_allSensorFeedback" << m_allSensorFeedbackInRadians.toString();

    return true;
}

const yarp::os::Stamp& RobotControlHelper::timeStamp() const
{
    return m_timeStamp;
}

yarp::os::Stamp& RobotControlHelper::timeStamp()
{
    return m_timeStamp;
}

const yarp::sig::Vector& RobotControlHelper::jointEncoders() const
{
    return m_encoderPositionFeedbackInRadians;
}

const yarp::sig::Vector& RobotControlHelper::analogSensors() const
{
    return m_analogSensorFeedbackInRadians;
}

const yarp::sig::Vector& RobotControlHelper::allSensors() const
{
    return m_allSensorFeedbackInRadians;
}

void RobotControlHelper::close()
{
    if (!switchToControlMode(VOCAB_CM_POSITION))
        yError() << "[RobotControlHelper::close] Unable to switch in position control.";

    if (!m_robotDevice.close())
        yError() << "[RobotControlHelper::close] Unable to close the device.";
}

int RobotControlHelper::getActuatedDoFs()
{
    return m_actuatedDOFs;
}

int RobotControlHelper::getNumberOfJoints()
{
    //return m_noAnalogSensor;
    return m_noAllSensor;

}

bool RobotControlHelper::getLimits(yarp::sig::Matrix& limits)
{
    if (!getFeedback())
    {
        yError() << "[RobotControlHelper::getLimits] Unable to get the feedback from the robot";
        return false;
    }

    // resize matrix
    limits.resize(m_actuatedDOFs, 2);

    double maxLimitInDegree, minLimitInDegree;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        // get position limits
        if (!m_limitsInterface->getLimits(i, &minLimitInDegree, &maxLimitInDegree))
        {
            if (m_isMandatory)
            {
                yError() << "[RobotControlHelper::getLimits] Unable get " << m_axesList[i]
                         << " joint limits.";
                return false;
            } else
            {
                limits(i, 0) = m_encoderPositionFeedbackInRadians(i);
                limits(i, 1) = m_encoderPositionFeedbackInRadians(i);
                yWarning()
                    << "[RobotControlHelper::getLimits] Unable get " << m_axesList[i]
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

bool RobotControlHelper::getVelocityLimits(yarp::sig::Matrix& limits)
{
    yInfo() << "RobotControlHelper::getVelocityLimits";
    if (!getFeedback())
    {
        yError()
            << "[RobotControlHelper::getVelocityLimits] Unable to get the feedback from the robot";
        return false;
    }
    // resize matrix
    limits.resize(m_actuatedDOFs, 2);

    double maxLimitInDegree, minLimitInDegree;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        // get position limits
        if (!m_limitsInterface->getVelLimits(i, &minLimitInDegree, &maxLimitInDegree))
        {
            yError() << "[RobotControlHelper::getVelocityLimits] Unable get " << m_axesList[i]
                     << " joint limits.";
            return false;

        } else
        {
            limits(i, 0) = iDynTree::deg2rad(minLimitInDegree);
            limits(i, 1) = iDynTree::deg2rad(maxLimitInDegree);
        }
    }
    return true;
}

bool RobotControlHelper::setJointReference(const yarp::sig::Vector& desiredValue)
{
    switch (m_controlMode)
    {
    case VOCAB_CM_POSITION_DIRECT:
        if (!setDirectPositionReferences(desiredValue))
        {
            yError() << "[RobotControlHelper::setJointReference] Unable to set the desired joint "
                        "position";
            return false;
        }
        break;

    case VOCAB_CM_VELOCITY:
        if (!setVelocityReferences(desiredValue))
        {
            yError() << "[RobotControlHelper::setJointReference] Unable to set the desired joint "
                        "velocity";
            return false;
        }
        break;

    default:
        yError() << "[RobotControlHelper::setJointReference] Unknown control mode.";
        return false;
    }

    return true;
}

bool RobotControlHelper::isVelocityControlUsed()
{
    return m_controlMode == VOCAB_CM_VELOCITY;
}
