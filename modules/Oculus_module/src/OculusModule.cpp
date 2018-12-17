/**
 * @file OculusModule.cpp
 * @authors  Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 *           Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#define _USE_MATH_DEFINES
#include <cmath>

// YARP
#include "yarp/os/LogStream.h"
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>

#include "OculusModule.hpp"
#include "Utils.hpp"

bool OculusModule::configureOculus()
{
    yarp::os::Property options;
    // move into the configuration file
    options.put("device", "transformClient");
    options.put("remote", "/transformServer");
    options.put("local", "/myTransform");

    if (!m_oculusDevice.open(options))
    {
        yError() << "[configureOculus] Unable to open transformClient device";
        return false;
    }

    // obtain the interface
    if (!m_oculusDevice.view(m_transformClient) || !m_transformClient)
    {
        yError() << "[configureOculus] Cannot obtain Transform client.";
        return false;
    }

    if (!m_transformClient->frameExists("mobile_base_body_link"))
    {
        yError() << "[configureOculus] No mobile_base_body_link frame.";
        return false;
    }
    m_joypadX = 0;
    m_joypadY = 0;

    return true;
}
bool OculusModule::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if (!m_controlModeInterface)
    {
        yError() << "[switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if (!m_controlModeInterface->setControlModes(controlModes.data()))
    {
        yError() << "[switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool OculusModule::setControlledJoints(const yarp::os::Searchable& rf)
{
    // get joints list from resource finder
    yarp::os::Value* axesListYarp;
    if (!rf.check("joints_list", axesListYarp))
    {
        yError() << "[setControlledJoints] Unable to find joints_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[setControlledJoints] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }
    return true;
}

bool OculusModule::configureRobot(const yarp::os::Searchable& config)
{
    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value* iCubPartsYarp;
    if (!config.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[configureRobot] Unable to find remote_control_boards into "
                    "config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[configureRobot] Unable to convert yarp list into a vector of "
                    "strings.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");
    YarpHelper::addVectorOfStringToProperty(options, "axesNames", m_axesList);

    // prepare the remotecontrolboards
    m_remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = m_remoteControlBoards.addList();
    for (auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    options.put("remoteControlBoards", m_remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + getName() + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    m_actuatedDOFs = m_axesList.size();
    yInfo() << "m_actuatedDOFs " << m_actuatedDOFs;

    // open the device
    if (!m_robotDevice.open(options))
    {
        yError() << "[configureRobot] Could not open remotecontrolboardremapper object.";
        return false;
    }

    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[configureRobot] Cannot obtain IEncoders interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionControl interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_limitsInterface) || !m_limitsInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    iTimed = nullptr;
    if (!m_robotDevice.view(iTimed))
    {
        yError() << "[configureRobot] Cannot obtain iTimed interface";
        return false;
    }

    m_qDesired.resize(m_actuatedDOFs);

    m_positionFeedbackInDegrees.resize(m_actuatedDOFs, 0.0);
    m_positionFeedbackInRadians.resize(m_actuatedDOFs, 0.0);
    m_velocityFeedbackInDegrees.resize(m_actuatedDOFs, 0.0);
    m_minJointsPosition.resize(m_actuatedDOFs, 0.0);
    m_maxJointsPosition.resize(m_actuatedDOFs, 0.0);

    // check if the robot is alive
    bool okPosition = false;
    bool okVelocity = false;
    for (int i = 0; i < 10 && !okPosition && !okVelocity; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());
        okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackInDegrees.data());

        if (!okPosition || !okVelocity)
            yarp::os::Time::delay(0.1);
    }
    if (!okPosition)
    {
        yError() << "[configure] Unable to read encoders (position).";
        return false;
    }

    // if(!okVelocity)
    // {
    //     yError() << "[configure] Unable to read encoders (velocity).";
    //     return false;
    // }

    double min, max;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        // get position limits
        if (!m_limitsInterface->getLimits(i, &min, &max))
        {
            yError() << "[configure] Unable get joints position limits.";
            return false;
        }

        yInfo() << "joint " << i << " min: " << min << "max : " << max;
        m_minJointsPosition(i) = min;
        m_maxJointsPosition(i) = max;
    }

    return true;
}

bool OculusModule::configure(yarp::os::ResourceFinder& rf)
{
    yarp::os::Value* value;

    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[configure] Empty configuration for the force torque sensors.";
        return false;
    }

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asDouble();

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }

    setName(name.c_str());

    if (!setControlledJoints(rf))
    {
        yError() << "[configure] Unable to set the controlled joints.";
        return false;
    }

    if (!configureRobot(rf))
    {
        yError() << "[configure] Unable to configure the robot";
        return false;
    }

    if (!configureOculus())
    {
        yError() << "[configure] Unable to configure the oculus";
        return false;
    }

    yarp::os::Bottle& joypadOptions = rf.findGroup("JOYPAD");
    // get the period
    if (joypadOptions.isNull())
        m_useVirtualizer = true;
    else
    {
        m_useVirtualizer = false;
        if (!YarpHelper::getDoubleFromSearchable(joypadOptions, "deadzone", m_deadzone))
        {
            yError() << "[Configure module] Unable to find parameter deadzone";
            return false;
        }
        if (!YarpHelper::getDoubleFromSearchable(joypadOptions, "fullscale", m_fullscale))
        {
            yError() << "[Configure module] Unable to find parameter deadzone";
            return false;
        }
        if (!YarpHelper::getDoubleFromSearchable(joypadOptions, "scale_X", m_scaleX))
        {
            yError() << "[Configure module] Unable to find parameter deadzone";
            return false;
        }
        if (!YarpHelper::getDoubleFromSearchable(joypadOptions, "scale_Y", m_scaleY))
        {
            yError() << "[Configure module] Unable to find parameter deadzone";
            return false;
        }
        m_useLeftStick = joypadOptions.check("use_left", yarp::os::Value("false")).asBool();
    }

    // configure head retargeting
    m_head = std::make_unique<HeadRetargeting>();
    yarp::os::Bottle& headOptions = rf.findGroup("HEAD_RETARGETING");
    if (!headOptions.isNull())
    {
        headOptions.append(generalOptions);
        if (!m_head->configure(headOptions))
        {
            yError() << "[configure] Unable to initialize the head retargeting.";
            return false;
        }

        m_useHead = true;
    } else
    {
        m_useHead = false;
    }

    // configure fingers retargeting
    m_leftHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
    if (!leftFingersOptions.isNull())
    {
        leftFingersOptions.append(generalOptions);
        if (!m_leftHandFingers->configure(leftFingersOptions))
        {
            yError() << "[configure] Unable to initialize the left fingers retargeting.";
            return false;
        }
        m_useLeftFingers = true;
    } else
        m_useLeftFingers = false;

    m_rightHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
    if (!rightFingersOptions.isNull())
    {
        rightFingersOptions.append(generalOptions);
        if (!m_rightHandFingers->configure(rightFingersOptions))
        {
            yError() << "[configure] Unable to initialize the right fingers retargeting.";
            return false;
        }
        m_useRightFingers = true;
    } else
        m_useRightFingers = false;

    // configure hands retargeting
    m_leftHand = std::make_unique<HandRetargeting>();
    yarp::os::Bottle& leftHandOptions = rf.findGroup("LEFT_HAND_RETARGETING");
    if (!leftHandOptions.isNull())
    {
        leftHandOptions.append(generalOptions);
        if (!m_leftHand->configure(leftHandOptions))
        {
            yError() << "[configure] Unable to initialize the left fingers retargeting.";
            return false;
        }
        m_useLeftHand = true;
    } else
        m_useLeftHand = false;

    m_rightHand = std::make_unique<HandRetargeting>();
    yarp::os::Bottle& rightHandOptions = rf.findGroup("RIGHT_HAND_RETARGETING");
    if (!rightHandOptions.isNull())
    {
        rightHandOptions.append(generalOptions);
        if (!m_rightHand->configure(rightHandOptions))
        {
            yError() << "[configure] Unable to initialize the right fingers retargeting.";
            return false;
        }
        m_useRightHand = true;
    } else
        m_useRightHand = false;

    // open ports
    std::string portName;
    if (!YarpHelper::getStringFromSearchable(rf, "leftHandPosePort", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_leftHandPosePort.open("/" + getName() + portName))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "rightHandPosePort", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_rightHandPosePort.open("/" + getName() + portName))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!m_imagesOrientationPort.open("/" + getName() + "/imagesOrientation:o"))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!m_robotOrientationPort.open("/" + getName() + "/robotOrientation:i"))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "joypadOculusPort", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_joypadOculusPort.open("/" + getName() + portName))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "oculusOrientationPort", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_oculusOrientationPort.open("/" + getName() + portName))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "playerOrientationPort", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_playerOrientationPort.open("/" + getName() + portName))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "joypadOculusAxisPort", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_joypadAxisPort.open("/" + getName() + portName))
    {
        yError() << "[configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "rpcPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_Joyrpc.open("/" + getName() + portName))
    {
        yError() << "[configure] " << portName << " port already open.";
        return false;
    }

    // switch control mode
    if (!switchToControlMode(VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "[close] Unable to switch in position direct control.";
        return false;
    }

    // initialize variables
    m_desiredHeadOrientation.resize(3, 0.0);

    m_fingerCommands.resize(13, 0.0);

    m_playerOrientation = 0;
    m_robotYaw = 0;

    m_loculus_T_rootFixed.resize(4, 4);
    m_roculus_T_rootFixed.resize(4, 4);

    // print status
    yInfo() << "Head retargeting" << m_useHead;
    yInfo() << "Left hand retargeting" << m_useLeftHand;
    yInfo() << "Right hand retargeting" << m_useRightHand;
    yInfo() << "Left finger retargeting" << m_useLeftFingers;
    yInfo() << "Right finger retargeting" << m_useRightFingers;
    return true;
}

double OculusModule::getPeriod()
{
    return m_dT;
}

bool OculusModule::close()
{
    if (!switchToControlMode(VOCAB_CM_POSITION))
    {
        yError() << "[close] Unable to switch in position control.";
        return false;
    }

    if (!m_robotDevice.close())
        yError() << "[close] Unable to close the device.";

    return true;
}

bool OculusModule::getFeedbacks()
{
    if (!m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data()))
    {
        yError() << "[getFeedbacks] Unable to get joint position";
        return false;
    }

    for (unsigned j = 0; j < m_actuatedDOFs; ++j)
    {
        m_positionFeedbackInRadians(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));
    }
    if (m_useVirtualizer)
    {
        yarp::sig::Vector* tmp = m_robotOrientationPort.read(false);
        if (tmp != NULL)
        {
            auto vector = *tmp;
            m_robotYaw = Angles::normalizeAngle(vector[0]);
        }
    }

    yarp::sig::Vector* axis = m_joypadAxisPort.read(false);
    if (axis != NULL)
    {
        if (m_useLeftStick)
        {
            m_joypadX = (*axis)[4];
            m_joypadY = (*axis)[5];
        } else
        {
            m_joypadX = (*axis)[6];
            m_joypadY = (*axis)[7];
        }
    }

    yarp::os::Bottle* desiredHeadOrientation = NULL;
    desiredHeadOrientation = m_oculusOrientationPort.read(false);
    if (desiredHeadOrientation != NULL)
    {
        for (int i = 0; i < desiredHeadOrientation->size(); i++)
            m_desiredHeadOrientation(i) = desiredHeadOrientation->get(i).asDouble();
    }

    yarp::os::Bottle* joypadButtons = NULL;
    joypadButtons = m_joypadOculusPort.read(false);
    if (joypadButtons != NULL)
    {
        for (int i = 0; i < joypadButtons->size(); i++)
            m_fingerCommands(i) = joypadButtons->get(i).asDouble();
    }

    yarp::sig::Vector* playerOrientation = NULL;
    playerOrientation = m_playerOrientationPort.read(false);
    if (playerOrientation != NULL)
        m_playerOrientation = (*playerOrientation)(0);

    yInfo() << "Player orientation: " << m_playerOrientation;

    if (!m_transformClient->getTransform("loculus", "mobile_base_body_link", m_loculus_T_rootFixed))
    {
        yError() << "Unable to evaluate the loculus to mobile_base_body_link "
                    "transformation";
        return false;
    }

    if (!m_transformClient->getTransform("roculus", "mobile_base_body_link", m_roculus_T_rootFixed))
    {
        yError() << "Unable to evaluate the roculus to mobile_base_body_link "
                    "transformation";
        return false;
    }

    return true;
}

bool OculusModule::setDirectPositionReferences(const yarp::sig::Vector& desiredPositions)
{
    if (m_positionDirectInterface == nullptr)
    {
        yError() << "[setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if (desiredPositions.size() != m_actuatedDOFs)
    {
        yError() << "[setDirectPositionReferences] Dimension mismatch between "
                    "desired position "
                 << "vector and the number of controlled joints.";
        return false;
    }

    if (!m_positionDirectInterface->setPositions(desiredPositions.data()))
    {
        yError() << "[setDirectPositionReferences] Error while setting the desired "
                    "position.";
        return false;
    }

    return true;
}

bool OculusModule::updateModule()
{

    // get the data from the ports
    if (!getFeedbacks())
    {
        yError() << "[updateModule] Unable to get the feedback";
        return false;
    }

    // check if everything is ok
    if (!m_transformClient->frameExists("roculus"))
    {
        yError() << "[updateModule] No right_hand_oculus frame.";
        return false;
    }

    if (!m_transformClient->frameExists("loculus"))
    {
        yError() << "[updateModule] No left_hand_oculus frame.";
        return false;
    }

    // head
    if (m_useHead)
    {
        m_head->setPlayerOrientation(m_playerOrientation * 180 / M_PI);
        m_head->setDesiredHeadOrientation(m_desiredHeadOrientation);
        m_head->evaluateHeadOrientationCorrected();
        auto tmp = m_head->getHeadOrientation();
        if (!m_qDesired.setSubvector(0, m_head->getHeadOrientation()))
        {
            yError() << "[updateModule] unable to add the desired head position";
            return false;
        }
    }

    // fingers
    // TODO remove magic numbers
    if (m_useRightFingers)
    {
        if (m_fingerCommands(0) == 1.0)
            m_rightHandFingers->closeHand();

        if (m_fingerCommands(1) == 1.0)
            m_rightHandFingers->openHand();

        if (!m_qDesired.setSubvector(3 + 7, m_rightHandFingers->fingerPosition()))
        {
            yError() << "[updateModule] Unable to add the desired left hand fingers.";
            return false;
        }
    }

    if (m_useLeftFingers)
    {
        if (m_fingerCommands(4) == 1.0)
            m_leftHandFingers->closeHand();

        if (m_fingerCommands(5) == 1.0)
            m_leftHandFingers->openHand();

        if (!m_qDesired.setSubvector(3, m_leftHandFingers->fingerPosition()))
        {
            yError() << "[updateModule] Unable to add the desired right hand fingers.";
            return false;
        }
    }
    // clear the finger commands
    // TODO REMOVE MAGIC NUMBER
    m_fingerCommands.resize(13, 0.0);
    // get the data from the ports
    if (!getFeedbacks())
    {
        yError() << "[updateModule] Unable to get the feedback";
        return false;
    }

    // check if joypads are ok
    if (!m_transformClient->frameExists("roculus"))
    {
        yError() << "[updateModule] No right_hand_oculus frame.";
        return false;
    }

    if (!m_transformClient->frameExists("loculus"))
    {
        yError() << "[updateModule] No left_hand_oculus frame.";
        return false;
    }

    yInfo() << "m_qDesired: " << m_qDesired.toString();

    // saturate move me in a function
    yarp::sig::Vector qDesiredSaturated(m_qDesired.size());
    double threshold = 5;
    for (int i = 0; i < m_qDesired.size(); i++)
    {
        if (m_qDesired(i) <= m_minJointsPosition(i))
            qDesiredSaturated(i) = m_minJointsPosition(i) + threshold;
        else if (m_qDesired(i) >= m_maxJointsPosition(i))
            qDesiredSaturated(i) = m_maxJointsPosition(i) - threshold;
        else
            qDesiredSaturated(i) = m_qDesired(i);
    }

    // move the robot
    if (!setDirectPositionReferences(m_qDesired))
    {
        yError() << "[updateModule] Unable to move the robot";
        return false;
    }

    // left hand
    if (m_useLeftHand)
    {
        yarp::sig::Vector& leftHandPose = m_leftHandPosePort.prepare();
        m_leftHand->setPlayerOrientation(m_playerOrientation);
        m_leftHand->setHandTransform(m_loculus_T_rootFixed);
        m_leftHand->evaluateHandToRootLinkTransform(leftHandPose);
        m_leftHandPosePort.write();
    }

    if (m_useRightHand)
    {
        // right hand
        yarp::sig::Vector& rightHandPose = m_rightHandPosePort.prepare();
        m_rightHand->setPlayerOrientation(m_playerOrientation);
        m_rightHand->setHandTransform(m_roculus_T_rootFixed);
        m_rightHand->evaluateHandToRootLinkTransform(rightHandPose);
        m_rightHandPosePort.write();
    }

    static yarp::os::Stamp ts;
    if (iTimed)
        ts = iTimed->getLastInputStamp();
    else
        ts.update();

    yarp::os::Bottle& imagesOrientation = m_imagesOrientationPort.prepare();
    imagesOrientation.clear();

    iDynTree::Rotation root_R_head = iDynTree::Rotation::RPY(m_positionFeedbackInRadians(0),
                                                             m_positionFeedbackInRadians(1),
                                                             m_positionFeedbackInRadians(2));
    iDynTree::Rotation inertial_R_root = iDynTree::Rotation::RotZ(m_robotYaw);
    iDynTree::Rotation inertial_R_head = inertial_R_root * root_R_head;
    iDynTree::Vector3 inertial_R_headRPY = inertial_R_head.asRPY();
    imagesOrientation.addDouble(iDynTree::rad2deg(inertial_R_headRPY(0)));
    imagesOrientation.addDouble(iDynTree::rad2deg(inertial_R_headRPY(1)));
    imagesOrientation.addDouble(iDynTree::rad2deg(inertial_R_headRPY(2)));

    for (int i = 3; i < 12; i++)
        imagesOrientation.addDouble(0);

    m_imagesOrientationPort.setEnvelope(ts);
    m_imagesOrientationPort.write();

    // use joypad
    if (!m_useVirtualizer)
    {
        double x, y;
        x = -m_scaleX * deadzone(m_joypadX);
        y = m_scaleY * deadzone(m_joypadY);

        std::swap(x, y);

        yarp::os::Bottle cmd, outcome;
        cmd.addString("setGoal");
        cmd.addDouble(x);
        cmd.addDouble(y);
        m_Joyrpc.write(cmd, outcome);
        yInfo() << "x is " << x << "and y is " << y;
    }

    return true;
}

double OculusModule::deadzone(const double& input)
{
    if (input >= 0)
    {
        if (input > m_deadzone)
            return (input - m_deadzone) / (m_fullscale - m_deadzone);
        else
            return 0.0;
    } else
    {
        if (input < -m_deadzone)
            return (input + m_deadzone) / (m_fullscale - m_deadzone);
        else
            return 0.0;
    }
}
