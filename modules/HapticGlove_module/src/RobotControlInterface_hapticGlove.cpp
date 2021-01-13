/**
 * @file HapticGloveRobotControlHelper.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <limits>
#include <algorithm>
#include <iterator>

// iDynTree
#include <iDynTree/Core/Utils.h>

#include <RobotControlInterface_hapticGlove.hpp>
#include <Utils.hpp>

using namespace HapticGlove;

bool RobotControlInterface::configure(const yarp::os::Searchable& config,
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
        yError() << "[RobotControlInterface::configure] Unable to find remote_control_boards into "
                    "config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[RobotControlInterface::configure] Unable to convert yarp list into a vector of "
                    "strings.";
        return false;
    }

    // get all icub senosry parts from the resource finder
    std::string iCubSensorPart;

    if (!YarpHelper::getStringFromSearchable(config, "remote_sensor_boards", iCubSensorPart))
    {
        yError() << "[RobotControlInterface::configure] Unable to find remote_sensor_boards into "
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
        yError() << "[RobotControlInterface::configure] Could not open analogsensorclient "
                    "object.";
        return false;
    }

    if (!m_analogDevice.view(m_AnalogSensorInterface) || !m_AnalogSensorInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IAnalogSensor interface";
        return false;
    }

    m_noAnalogSensor = config.check("noAnalogSensor", yarp::os::Value(15)).asInt();
    yInfo() << "m_noAnalogSensor " << m_noAnalogSensor;

    m_noAllSensor = config.check("noAllSensor", yarp::os::Value(17)).asInt(); // 5*3 analog sensors+ 1 encoder thumb oppose + 1 encoder hand_finger
    yInfo() << "m_noAllSensor " << m_noAllSensor;

    m_noAllSensor=0;

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
        yError() << "[RobotControlInterface::configure] Unable to find axis_list into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[RobotControlInterface::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }

    // add the list of axis and associated analog sensors
    yarp::os::Value* analogListYarp;
    if (!config.check("analog_list", analogListYarp))
    {
        yError() << "[RobotControlInterface::configure] Unable to find analog_list into config file.";
        return false;
    }
    std::vector<std::string> analogList;
    if (!YarpHelper::yarpListToStringVector(analogListYarp, analogList))
    {
        yError() << "[RobotControlInterface::configure] Unable to convert yarp analog list into a "
                    "vector of strings.";
        return false;
    }

    yarp::os::Value* jointListYarp;
    if (!config.check("joint_list", jointListYarp))
    {
        yError() << "[RobotControlInterface::configure] Unable to find joint_list into config file.";
        return false;
    }
    std::vector<std::string> jointList;
    if (!YarpHelper::yarpListToStringVector(jointListYarp, jointList))
    {
        yError() << "[RobotControlInterface::configure] Unable to convert yarp joint list into a "
                    "vector of strings.";
        return false;
    }

    yarp::os::Value* jointFeedbackTypeListYarp;

    if (!config.check("joint_fb_type", jointFeedbackTypeListYarp))
    {
        yError() << "[RobotControlInterface::configure] Unable to find joint_fb_type into config file.";
        return false;
    }
    std::vector<std::string> jointFeedbackTypeList;
    if (!YarpHelper::yarpListToStringVector(jointFeedbackTypeListYarp, jointFeedbackTypeList))
    {
        yError() << "[RobotControlInterface::configure] Unable to convert yarp joint fb type list into a "
                    "vector of strings.";
        return false;
    }
    std::vector<bool> jointFeedbackIsAnalog;
    for(std::vector<std::string>::iterator it = jointFeedbackTypeList.begin() ; it != jointFeedbackTypeList.end(); ++it)
    {
        jointFeedbackIsAnalog.push_back(*it=="analog");
    }

    for (size_t i=0; i<m_axesList.size();i++)
    {
        axisSensorData axisInfoObj;
        axisInfoObj.axisName=m_axesList[i];
        yarp::os::Value* axisJointListYarp;
        if (!config.check(axisInfoObj.axisName, axisJointListYarp))
        {
            yError() << "[RobotControlInterface::configure] Unable to find "<<axisInfoObj.axisName<<"into config file.";
            return false;
        }

        std::vector<std::string> axisJointList; // related joints to the axisName
        if (!YarpHelper::yarpListToStringVector(axisJointListYarp, axisJointList))
        {
            yError() << "[RobotControlInterface::configure] Unable to convert yarp axisAnalog list into a "
                        "vector of strings.";
            return false;
        }

        m_noAllSensor+=axisJointList.size(); // add the number of associated joint sensors to the total number of active sensors


        // *it: joint name related to the axisName
        for (std::vector<std::string>::iterator it = axisJointList.begin() ; it != axisJointList.end(); ++it)
        {
            m_activatedJointList.push_back(*it);

            auto elementJoint = std::find(std::begin(jointList), std::end(jointList), *it );
            if (elementJoint == std::end(jointList)) {
                yError() << "[RobotControlInterface::configure] Unable to find "<<*it  <<" in joint vector list ";
                "vector of strings.";
                return false;
            }
            size_t indexJoint= elementJoint -jointList.begin();
            // All the joint moved by an axis is measured with one senory feedback type, i.e., either analog or encoder
            if(jointFeedbackIsAnalog[indexJoint])
            {
                axisInfoObj.useAnalog=true;
            }

            if(axisInfoObj.useAnalog)
            {
                auto elementAnalog= std::find(std::begin(analogList), std::end(analogList), *it );
                if (elementAnalog== std::end(analogList)) {
                    yError()<<"[RobotControlInterface::configure] the joint name ("<< *it<<" )is not found in the analog list, but it should be an analog sensor.";
                    return false;
                }
                size_t indexAnalog= elementAnalog-analogList.begin();
                axisInfoObj.relatedAnalogSensorsIndex.push_back(indexAnalog);
            }
        }
        m_axisInfoList.push_back(axisInfoObj);
    }


    //
    m_joints_min_boundary.resize(m_noAnalogSensor, 0.0);
    m_joints_max_boundary.resize(m_noAnalogSensor, 0.0);
    m_sensors_min_boundary.resize(m_noAnalogSensor, 0.0);
    m_sensors_max_boundary.resize(m_noAnalogSensor, 0.0);
    m_sensors_raw2Degree_scaling.resize(m_noAnalogSensor, 0.0);

    // get the joints limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
                config, "joints_min_boundary", m_joints_min_boundary))
    {
        yError() << "RobotControlInterface::configure] unable to get the minimum boundary of the joints limits.";
        return false;
    }

    if (!YarpHelper::getYarpVectorFromSearchable(
                config, "joints_max_boundary", m_joints_max_boundary))
    {
        yError() << "RobotControlInterface::configure] unable to get the maximum boundary of the "
                    "joints limits.";
        return false;
    }

    // get the sensors limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
                config, "sensors_min_boundary", m_sensors_min_boundary))
    {
        yError() << "RobotControlInterface::configure] unable to get the minimum boundary of the "
                    "joints limits.";
        return false;
    }

    // get the sensors limits boundaries
    if (!YarpHelper::getYarpVectorFromSearchable(
                config, "sensors_max_boundary", m_sensors_max_boundary))
    {
        yError() << "RobotControlInterface::configure] unable to get the maximum boundary of the "
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
        yError() << "RobotControlInterface::configure] unable to get the maximum boundary of the "
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
        yError() << "[RobotControlInterface::configure] Could not open remotecontrolboardremapper "
                    "object.";
        return false;
    }

    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IEncoders interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IPositionControl interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IVelocityInterface interface";
        return false;
    }

    if (!m_robotDevice.view(m_limitsInterface) || !m_limitsInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain IControlMode interface";
        return false;
    }

    if (!m_robotDevice.view(m_timedInterface) || !m_timedInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain iTimed interface";
        return false;
    }
    m_currentInterface;
    if (!m_robotDevice.view(m_currentInterface) || !m_currentInterface)
    {
        yError() << "[RobotControlInterface::configure] Cannot obtain ICurrentControl interface";
        return false;
    }

    m_desiredJointValue.resize(m_actuatedDOFs);
    m_encoderPositionFeedbackInDegrees.resize(m_actuatedDOFs);
    m_encoderPositionFeedbackInRadians.resize(m_actuatedDOFs);
    m_encoderVelocityFeedbackInDegrees.resize(m_actuatedDOFs);
    m_encoderVelocityFeedbackInRadians.resize(m_actuatedDOFs);

    m_desiredCurrent.resize(m_actuatedDOFs);
    m_desiredCurrentInterface.resize(m_actuatedDOFs);
    m_currentFeedback.resize(m_actuatedDOFs);

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
        yError() << "[RobotControlInterface::configure] Unable to read encoders (position).";
        return false;
    }

    if (!switchToControlMode(m_controlMode))
    {
        yError() << "[RobotControlInterface::configure] Unable to switch the control mode";
        return false;
    }
    return true;
}

bool RobotControlInterface::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if (!m_controlModeInterface)
    {
        yError() << "[RobotControlInterface::switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if (!m_controlModeInterface->setControlModes(controlModes.data()) && m_isMandatory)
    {
        yError()
                << "[RobotControlInterface::switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool RobotControlInterface::setDirectPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionDirectInterface == nullptr)
    {
        yError()
                << "[RobotControlInterface::setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlInterface::setDirectPositionReferences] Dimension mismatch between "
                    "desired position vector and the number of controlled joints.";
        return false;
    }

    // convert radiant to degree
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredPosition(i));

    // set desired position
    if (!m_positionDirectInterface->setPositions(m_desiredJointValue.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::setDirectPositionReferences] Error while setting the "
                    "desired position.";
        return false;
    }

    return true;
}

bool RobotControlInterface::setPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionInterface == nullptr)
    {
        yError() << "[RobotControlInterface::setPositionReferences] Position I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlInterface::setDirectPositionReferences] Dimension mismatch between "
                    "desired position vector and the number of controlled joints.";
        return false;
    }

    // convert radiant to degree
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredPosition(i));

    // set desired position
    if (!m_positionInterface->positionMove(m_desiredJointValue.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::setPositionReferences] Error while setting the "
                    "desired position.";
        return false;
    }

    return true;
}

bool RobotControlInterface::setVelocityReferences(const yarp::sig::Vector& desiredVelocity)
{
    if (m_velocityInterface == nullptr)
    {
        yError() << "[RobotControlInterface::setVelocityReferences] Velocity I/F not ready.";
        return false;
    }

    if (desiredVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlInterface::setVelocityReferences] Dimension mismatch between "
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
        yError() << "[RobotControlInterface::setVelocityReferences] Error while setting the desired "
                    "acceleration.";
        return false;
    }

    if (!m_velocityInterface->velocityMove(m_desiredJointValue.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::setVelocityReferences] Error while setting the desired "
                    "velocity.";
        return false;
    }
    return true;
}

bool RobotControlInterface::setCurrentReferences(const yarp::sig::Vector& desiredCurrent)
{
    if (m_currentInterface== nullptr)
    {
        yError() << "[RobotControlInterface::setCurrentReferences] Current I/F not ready.";
        return false;
    }

    if (desiredCurrent.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlInterface::setCurrentReferences] Dimension mismatch between "
                    "desired current vector and the number of controlled joints.";
        return false;
    }

    // update the desired current values
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredCurrent(i) = desiredCurrent(i);

    // set desired current
    if (!m_currentInterface->setRefCurrents(m_desiredCurrent.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::setCurrentReferences] Error while setting the "
                    "desired current.";
        return false;
    }
    return true;
}

void RobotControlInterface::updateTimeStamp()
{
    if (m_timedInterface)
        m_timeStamp = m_timedInterface->getLastInputStamp();
    else
        m_timeStamp.update();
}

bool RobotControlInterface::getFeedback()
{
    if (!m_encodersInterface->getEncoders(m_encoderPositionFeedbackInDegrees.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::getFeedbacks] Unable to get joint position";
        return false;
    }

    for (unsigned j = 0; j < m_actuatedDOFs; ++j)
        m_encoderPositionFeedbackInRadians(j) = iDynTree::deg2rad(m_encoderPositionFeedbackInDegrees(j));

    if (!m_encodersInterface->getEncoderSpeeds(m_encoderVelocityFeedbackInDegrees.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::getFeedbacks] Unable to get Axis velocity feedback";
        return false;
    }

    for (unsigned j = 0; j < m_actuatedDOFs; ++j)
        m_encoderVelocityFeedbackInRadians(j) = iDynTree::deg2rad(m_encoderVelocityFeedbackInDegrees(j));



    if (!(m_AnalogSensorInterface->read(m_analogSensorFeedbackRaw) == yarp::dev::IAnalogSensor::AS_OK))
    {
        yError() << "[RobotControlInterface::getFeedbacks] Unable to get analog sensor data";
        return false;
    }
    if (!getCalibratedFeedback())
    {
        yError() << "[RobotControlInterface::getFeedbacks] Unable to get the calibrated analog "
                    "sensor data";
        return false;
    }

    if(!setAllJointsFeedback())
    {
        yError() << "[RobotControlInterface::getFeedbacks] Unable to set all the interested joints feedback"
                    "sensor data";
        return false;
    }

    if (!m_currentInterface->getCurrents(m_currentFeedback.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::getFeedbacks] Unable to get motor current feedbacks";
        return false;
    }

    if (!m_currentInterface->getRefCurrents(m_desiredCurrentInterface.data()) && m_isMandatory)
    {
        yError() << "[RobotControlInterface::getFeedbacks] Unable to get the motor desired current from the interface";
        return false;
    }
    return true;
}

bool RobotControlInterface::getCalibratedFeedback()
{

    for (unsigned j = 0; j < m_noAnalogSensor; ++j)
    {
        m_analogSensorFeedbackInDegrees(j) = m_joints_min_boundary(j)
                + m_sensors_raw2Degree_scaling(j)
                * (m_analogSensorFeedbackRaw(m_analogSensorFeedbackSelected(j))
                   - m_sensors_min_boundary(j)); // TOCHECK
        m_analogSensorFeedbackInRadians(j) = iDynTree::deg2rad(m_analogSensorFeedbackInDegrees(j));
    }

    return true;
}

bool RobotControlInterface::setAllJointsFeedback()
{


    size_t idx=0;
    for (size_t i =0; i< m_axisInfoList.size() ;++i)
    {
        if(m_axisInfoList[i].useAnalog)
        {
            // "it" is the pointer to ananlog Index
            for(std::vector<int>::iterator it = m_axisInfoList[i].relatedAnalogSensorsIndex.begin() ; it != m_axisInfoList[i].relatedAnalogSensorsIndex.end(); ++it)
            {
                m_allSensorFeedbackInRadians(idx)=m_analogSensorFeedbackInRadians(*it);
                idx++;
            }
        }
        else
        {
            m_allSensorFeedbackInRadians(idx)=m_encoderPositionFeedbackInRadians(i);
            idx++;
        }
    }

    //    m_allSensorFeedbackInRadians(0)=m_encoderPositionFeedbackInRadians(0);
    //    for(unsigned j = 0; j < m_noAnalogSensor; ++j)
    //    {
    //        m_allSensorFeedbackInRadians(j+1)=m_analogSensorFeedbackInRadians(j);
    //    }

    return true;
}

const yarp::os::Stamp& RobotControlInterface::timeStamp() const
{
    return m_timeStamp;
}

yarp::os::Stamp& RobotControlInterface::timeStamp()
{
    return m_timeStamp;
}

const yarp::sig::Vector& RobotControlInterface::jointEncoders() const
{
    return m_encoderPositionFeedbackInRadians;
}

const yarp::sig::Vector& RobotControlInterface::jointEncodersSpeed() const
{
    return m_encoderVelocityFeedbackInRadians;
}

const yarp::sig::Vector& RobotControlInterface::analogSensors() const
{
    return m_analogSensorFeedbackInRadians;
}

const yarp::sig::Vector& RobotControlInterface::allSensors() const
{
    return m_allSensorFeedbackInRadians;
}

const yarp::sig::Vector& RobotControlInterface::motorCurrents() const
{
    return m_currentFeedback;
}

const yarp::sig::Vector& RobotControlInterface::motorCurrentReference() const
{
    return m_desiredCurrentInterface;
}

void RobotControlInterface::close()
{
    if (!switchToControlMode(VOCAB_CM_POSITION))
        yError() << "[RobotControlInterface::close] Unable to switch in position control.";

    if (!m_robotDevice.close())
        yError() << "[RobotControlInterface::close] Unable to close the device.";
}

int RobotControlInterface::getActuatedDoFs()
{
    return m_actuatedDOFs;
}

int RobotControlInterface::getNumberOfJoints()
{
    //return m_noAnalogSensor;
    return m_noAllSensor;

}

bool RobotControlInterface::getLimits(yarp::sig::Matrix& limits)
{
    if (!getFeedback())
    {
        yError() << "[RobotControlInterface::getLimits] Unable to get the feedback from the robot";
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
                yError() << "[RobotControlInterface::getLimits] Unable get " << m_axesList[i]
                            << " joint limits.";
                return false;
            } else
            {
                limits(i, 0) = m_encoderPositionFeedbackInRadians(i);
                limits(i, 1) = m_encoderPositionFeedbackInRadians(i);
                yWarning()
                        << "[RobotControlInterface::getLimits] Unable get " << m_axesList[i]
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

bool RobotControlInterface::getVelocityLimits(yarp::sig::Matrix& limits)
{
    yInfo() << "RobotControlInterface::getVelocityLimits";
    if (!getFeedback())
    {
        yError()
                << "[RobotControlInterface::getVelocityLimits] Unable to get the feedback from the robot";
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
            yError() << "[RobotControlInterface::getVelocityLimits] Unable get " << m_axesList[i]
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

bool RobotControlInterface::setJointReference(const yarp::sig::Vector& desiredValue)
{
    switch (m_controlMode)
    {
    case VOCAB_CM_POSITION_DIRECT:
        if (!setDirectPositionReferences(desiredValue))
        {
            yError() << "[RobotControlInterface::setJointReference] Unable to set the desired joint "
                        "position";
            return false;
        }
        break;

    case VOCAB_CM_VELOCITY:
        if (!setVelocityReferences(desiredValue))
        {
            yError() << "[RobotControlInterface::setJointReference] Unable to set the desired joint "
                        "velocity";
            return false;
        }
        break;

    default:
        yError() << "[RobotControlInterface::setJointReference] Unknown control mode.";
        return false;
    }

    return true;
}

bool RobotControlInterface::isVelocityControlUsed()
{
    return m_controlMode == VOCAB_CM_VELOCITY;
}
