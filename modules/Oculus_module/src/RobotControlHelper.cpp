/**
 * @file RobotControlHelper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/Utils.h>

#include <RobotControlHelper.hpp>
#include <Utils.hpp>

bool RobotControlHelper::configure(const yarp::os::Searchable& config, const std::string& name)
{
    m_isActive = true;

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
        m_isActive = false;
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a vector of "
                    "strings.";
        m_isActive = false;
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    std::vector<std::string> axesList;
    yarp::os::Value* axesListYarp;
    if (!config.check("joints_list", axesListYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find joints_list into config file.";
        m_isActive = false;
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, axesList))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        m_isActive = false;
        return false;
    }

    options.put("device", "remotecontrolboardremapper");
    YarpHelper::addVectorOfStringToProperty(options, "axesNames", axesList);

    // prepare the remotecontrolboards
    yarp::os::Bottle remoteControlBoards;
    remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    options.put("remoteControlBoards", remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    m_actuatedDOFs = axesList.size();

    // open the device
    if (!m_robotDevice.open(options))
    {
        yError() << "[RobotControlHelper::configure] Could not open remotecontrolboardremapper "
                    "object.";
        m_isActive = false;
        return false;
    }

    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IEncoders interface";
        m_isActive = false;
        return false;
    }

    if (!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionControl interface";
        m_isActive = false;
        return false;
    }

    if (!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        m_isActive = false;
        return false;
    }

    if (!m_robotDevice.view(m_limitsInterface) || !m_limitsInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        m_isActive = false;
        return false;
    }

    if (!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IControlMode interface";
        m_isActive = false;
        return false;
    }

    if (!m_robotDevice.view(m_timedInterface) || !m_timedInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain iTimed interface";
        m_isActive = false;
        return false;
    }

    m_desiredPositionInDegrees.resize(m_actuatedDOFs);
    m_positionFeedbackInDegrees.resize(m_actuatedDOFs);
    m_positionFeedbackInRadians.resize(m_actuatedDOFs);
    m_minJointsPosition.resize(m_actuatedDOFs);
    m_maxJointsPosition.resize(m_actuatedDOFs);

    // check if the robot is alive
    bool okPosition = false;
    for (int i = 0; i < 10 && !okPosition; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());

        if (!okPosition)
            yarp::os::Time::delay(0.1);
    }
    if (!okPosition)
    {
        yError() << "[configure] Unable to read encoders (position).";
        m_isActive = false;
        return false;
    }

    for (int i = 0; i < m_actuatedDOFs; i++)
        // get position limits
        if (!m_limitsInterface->getLimits(i, &m_minJointsPosition(i), &m_maxJointsPosition(i)))
        {
            yError() << "[configure] Unable get joints position limits.";
            m_isActive = false;
            return false;
        }

    return true;
}

bool RobotControlHelper::switchToControlMode(const int& controlMode)
{
    // if the part is not active skip
    if (!m_isActive)
        return true;

    // check if the control interface is ready
    if (!m_controlModeInterface)
    {
        yError() << "[RobotControlHelper::switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if (!m_controlModeInterface->setControlModes(controlModes.data()))
    {
        yError()
            << "[RobotControlHelper::switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool RobotControlHelper::setDirectPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (!m_isActive)
        return true;

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

    double threshold = 5;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        m_desiredPositionInDegrees(i) = iDynTree::rad2deg(desiredPosition(i));

        // check if the position is outside the limit
        if (m_desiredPositionInDegrees(i) <= m_minJointsPosition(i))
            m_desiredPositionInDegrees(i) = m_minJointsPosition(i) + threshold;
        else if (m_desiredPositionInDegrees(i) >= m_maxJointsPosition(i))
            m_desiredPositionInDegrees(i) = m_maxJointsPosition(i) - threshold;
    }

    if (!m_positionDirectInterface->setPositions(m_desiredPositionInDegrees.data()))
    {
        yError() << "[RobotControlHelper::setDirectPositionReferences] Error while setting the "
                    "desired position.";
        return false;
    }

    return true;
}

void RobotControlHelper::updateTimeStamp()
{
    if (!m_isActive)
        return;

    if (m_timedInterface)
        m_timeStamp = m_timedInterface->getLastInputStamp();
    else
        m_timeStamp.update();
}

bool RobotControlHelper::getFeedback()
{
    if (!m_isActive)
        return true;

    if (!m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data()))
    {
        yError() << "[RobotControlHelper::getFeedbacks] Unable to get joint position";
        return false;
    }

    for (unsigned j = 0; j < m_actuatedDOFs; ++j)
        m_positionFeedbackInRadians(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));

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
    return m_positionFeedbackInRadians;
}

void RobotControlHelper::close()
{
    if (!switchToControlMode(VOCAB_CM_POSITION))
    {
        yError() << "[close] Unable to switch in position control.";
    }

    if (!m_robotDevice.close())
        yError() << "[close] Unable to close the device.";
}

int RobotControlHelper::getDoFs()
{
    return m_actuatedDOFs;
}
