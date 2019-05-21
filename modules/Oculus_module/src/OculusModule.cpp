/**
 * @file OculusModule.cpp
 * @authors  Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 *           Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <OculusModule.hpp>
#include <Utils.hpp>

bool OculusModule::configureTranformClient(const yarp::os::Searchable& config)
{
    yarp::os::Property options;
    options.put("device", "transformClient");
    options.put("remote", "/transformServer");
    options.put("local", "/" + getName() + "/transformClient");

    if (!m_transformClientDevice.open(options))
    {
        yError() << "[OculusModule::configureTranformClient] Unable to open transformClient device";
        return false;
    }

    // obtain the interface
    if (!m_transformClientDevice.view(m_frameTransformInterface) || !m_frameTransformInterface)
    {
        yError() << "[OculusModule::configureTranformClient] Cannot obtain Transform client.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(config, "root_frame_name", m_rootFrameName))
    {
        yError() << "[OculusModule::configureTranformClient] Cannot obtain Transform client.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(config, "head_frame_name", m_headFrameName))
    {
        yWarning() << "[OculusModule::configureTranformClient] Seems that the head "
                      "orientation is not streamed through the transform server.";

        // the oculus headset orientation is actually streamed through a yarp port and
        // not using the transform server. In a future implementation this port
        // will be removed
        std::string portName;
        if (!YarpHelper::getStringFromSearchable(config, "oculusOrientationPort", portName))
        {
            yError() << "[OculusModule::configureTranformClient] Unable to get a string from a "
                        "searchable";
            return false;
        }

        if (!m_oculusOrientationPort.open("/" + getName() + portName))
        {
            yError() << "[OculusModule::configureTranformClient] Unable to open the port "
                     << "/" << getName() << portName;
            return false;
        }
    }

    if (!YarpHelper::getStringFromSearchable(config, "left_hand_frame_name", m_leftHandFrameName))
    {
        yError() << "[OculusModule::configureTranformClient] Cannot obtain Transform client.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(config, "right_hand_frame_name", m_rightHandFrameName))
    {
        yError() << "[OculusModule::configureTranformClient] Cannot obtain Transform client.";
        return false;
    }

    m_oculusRoot_T_lOculus.resize(4, 4);
    m_oculusRoot_T_rOculus.resize(4, 4);
    m_oculusRoot_T_headOculus.resize(4, 4);

    return true;
}

bool OculusModule::configureJoypad(const yarp::os::Searchable& config)
{
    yarp::os::Property options;
    options.put("device", "JoypadControlClient");
    options.put("remote", "/joypadDevice/Oculus");
    options.put("local", "/" + getName() + "/joypadControlClient");

    if (!m_joypadDevice.open(options))
    {
        yError() << "[OculusModule::configureJoypad] Unable to open the polydriver.";
        return false;
    }

    // get the interface
    if (!m_joypadDevice.view(m_joypadControllerInterface) || !m_joypadControllerInterface)
    {
        yError() << "[OculusModule::configureJoypad] Unable to attach JoypadController interface "
                    "to the PolyDriver object";
        return false;
    }

    m_useVirtualizer = !(config.check("move_icub_using_joypad", yarp::os::Value(false)).asBool());
    if (!m_useVirtualizer)
    {
        m_useVirtualizer = false;
        if (!YarpHelper::getDoubleFromSearchable(config, "deadzone", m_deadzone))
        {
            yError() << "[OculusModule::configureJoypad] Unable to find parameter deadzone";
            return false;
        }
        if (!YarpHelper::getDoubleFromSearchable(config, "fullscale", m_fullscale))
        {
            yError() << "[OculusModule::configureJoypad] Unable to find parameter fullscale";
            return false;
        }
        if (!YarpHelper::getDoubleFromSearchable(config, "scale_X", m_scaleX))
        {
            yError() << "[OculusModule::configureJoypad] Unable to find parameter scale_X";
            return false;
        }
        if (!YarpHelper::getDoubleFromSearchable(config, "scale_Y", m_scaleY))
        {
            yError() << "[OculusModule::configureJoypad] Unable to find parameter scale_Y";
            return false;
        }

        // set the index of the axis according to the OVRheadset yarp device
        bool useLeftStick = config.check("use_left", yarp::os::Value("false")).asBool();
        m_xJoypadIndex = useLeftStick ? 4 : 6;
        m_yJoypadIndex = useLeftStick ? 5 : 7;
    }

    m_squeezeLeftIndex = 0;
    m_squeezeRightIndex = 1;

    m_releaseLeftIndex = 2;
    m_releaseRightIndex = 3;

    m_startWalkingIndex = 4; // X button
    m_prepareWalkingIndex = 0; // A button

    return true;
}

bool OculusModule::configureOculus(const yarp::os::Searchable& config)
{
    if (!configureTranformClient(config))
    {
        yError() << "[OculusModule::configureOculus] Unable to configure the transform client";
        return false;
    }

    if (!configureJoypad(config))
    {
        yError() << "[OculusModule::configureOculus] Unable to configure the joypad client";
        return false;
    }

    return true;
}

bool OculusModule::configure(yarp::os::ResourceFinder& rf)
{
    yarp::os::Value* value;

    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError()
            << "[OculusModule::configure] Empty configuration for the OculusModule application.";
        return false;
    }

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asDouble();

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    m_useXsens = generalOptions.check("useXsens", yarp::os::Value(false)).asBool();
    yInfo() << "Teleoperation uses Xsens: " << m_useXsens;

    yarp::os::Bottle& oculusOptions = rf.findGroup("OCULUS");
    if (!configureOculus(oculusOptions))
    {
        yError() << "[OculusModule::configure] Unable to configure the oculus";
        return false;
    }

    // configure head retargeting
    m_head = std::make_unique<HeadRetargeting>();
    yarp::os::Bottle& headOptions = rf.findGroup("HEAD_RETARGETING");
    headOptions.append(generalOptions);
    if (!m_head->configure(headOptions, getName()))
    {
        yError() << "[OculusModule::configure] Unable to initialize the head retargeting.";
        return false;
    }

    // configure torso retargeting, iff we use Xsens

    yInfo() << "[OculusModule::configure] initialize the torso!";
    if (m_useXsens)
    {
        m_torso = std::make_unique<TorsoRetargeting>();
        yarp::os::Bottle& torsoOptions = rf.findGroup("TORSO_RETARGETING");
        torsoOptions.append(generalOptions);
        if (!m_torso->configure(torsoOptions, getName()))
        {
            yError() << "[OculusModule::configure] Unable to initialize the torso retargeting.";
            return false;
        }
    }
    yInfo() << "[OculusModule::configure] finish the torso!";

    // configure fingers retargeting
    m_leftHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
    leftFingersOptions.append(generalOptions);
    if (!m_leftHandFingers->configure(leftFingersOptions, getName()))
    {
        yError() << "[OculusModule::configure] Unable to initialize the left fingers retargeting.";
        return false;
    }

    m_rightHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
    rightFingersOptions.append(generalOptions);
    if (!m_rightHandFingers->configure(rightFingersOptions, getName()))
    {
        yError() << "[OculusModule::configure] Unable to initialize the right fingers retargeting.";
        return false;
    }

    // configure hands retargeting
    m_leftHand = std::make_unique<HandRetargeting>();
    yarp::os::Bottle& leftHandOptions = rf.findGroup("LEFT_HAND_RETARGETING");
    leftHandOptions.append(generalOptions);
    if (!m_leftHand->configure(leftHandOptions))
    {
        yError() << "[OculusModule::configure] Unable to initialize the left fingers retargeting.";
        return false;
    }

    m_rightHand = std::make_unique<HandRetargeting>();
    yarp::os::Bottle& rightHandOptions = rf.findGroup("RIGHT_HAND_RETARGETING");
    rightHandOptions.append(generalOptions);
    if (!m_rightHand->configure(rightHandOptions))
    {
        yError() << "[OculusModule::configure] Unable to initialize the right fingers retargeting.";
        return false;
    }

    // open ports
    std::string portName;
    if (!YarpHelper::getStringFromSearchable(rf, "leftHandPosePort", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_leftHandPosePort.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "rightHandPosePort", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_rightHandPosePort.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] Unable to open the port " << portName;
        return false;
    }

    if (!m_imagesOrientationPort.open("/" + getName() + "/imagesOrientation:o"))
    {
        yError() << "[OculusModule::configure] Unable to open the port " << portName;
        return false;
    }
    if (!m_robotOrientationPort.open("/" + getName() + "/robotOrientation:i"))
    {
        yError() << "[OculusModule::configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "playerOrientationPort", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_playerOrientationPort.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "rpcWalkingPort_name", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_rpcWalkingClient.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] " << portName << " port already open.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "rpcVirtualizerPort_name", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_rpcVirtualizerClient.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] " << portName << " port already open.";
        return false;
    }

    m_playerOrientation = 0;
    m_robotYaw = 0;

    m_state = OculusFSM::Configured;

    return true;
}

double OculusModule::getPeriod()
{
    return m_dT;
}

bool OculusModule::close()
{
    // close devices
    m_head->controlHelper()->close();
    m_rightHandFingers->controlHelper()->close();
    m_leftHandFingers->controlHelper()->close();

    if (m_useXsens)
    {
        m_torso->controlHelper()->close();
    }

    m_joypadDevice.close();
    m_transformClientDevice.close();

    return true;
}

double OculusModule::evaluateDesiredFingersVelocity(unsigned int squeezeIndex,
                                                    unsigned int releaseIndex)
{
    double releaseFingersVelocity, squeezeFingersVelocity;
    m_joypadControllerInterface->getAxis(squeezeIndex, squeezeFingersVelocity);
    m_joypadControllerInterface->getAxis(releaseIndex, releaseFingersVelocity);

    if (squeezeFingersVelocity > releaseFingersVelocity)
        return squeezeFingersVelocity;
    else if (squeezeFingersVelocity < releaseFingersVelocity)
        return -releaseFingersVelocity;
    else
        return 0;
}

bool OculusModule::getTransforms()
{
    if (!m_useXsens)
    {
        // check if everything is ok
        if (!m_frameTransformInterface->frameExists(m_rootFrameName))
        {
            yError() << "[OculusModule::getTransforms] No " << m_rootFrameName << " frame.";
            return false;
        }

        if (!m_frameTransformInterface->frameExists(m_headFrameName))
        {
            // head
            yarp::os::Bottle* desiredHeadOrientation = NULL;

            iDynTree::Vector3 desiredHeadOrientationVector;
            desiredHeadOrientation = m_oculusOrientationPort.read(false);
            if (desiredHeadOrientation != NULL)
            {
                for (int i = 0; i < desiredHeadOrientation->size(); i++)
                    desiredHeadOrientationVector(i)
                        = iDynTree::deg2rad(desiredHeadOrientation->get(i).asDouble());

                // Notice that the data coming from the port are written in the following order:
                // [ pitch, -roll, yaw].
                iDynTree::toEigen(m_oculusRoot_T_headOculus).block(0, 0, 3, 3)
                    = iDynTree::toEigen(iDynTree::Rotation::RPY(-desiredHeadOrientationVector(1),
                                                                desiredHeadOrientationVector(0),
                                                                desiredHeadOrientationVector(2)));
            }
        } else
        {
            if (!m_frameTransformInterface->getTransform(
                    m_headFrameName, m_rootFrameName, m_oculusRoot_T_headOculus))
            {
                yError() << "[OculusModule::getTransforms] Unable to evaluate the "
                         << m_headFrameName << " to " << m_rootFrameName << "transformation";
                return false;
            }
        }

        if (!m_frameTransformInterface->frameExists(m_leftHandFrameName))
        {

            yError() << "[OculusModule::getTransforms] No " << m_leftHandFrameName << " frame.";
            return false;
        }

        if (!m_frameTransformInterface->frameExists(m_rightHandFrameName))
        {
            yError() << "[OculusModule::getTransforms] No " << m_rightHandFrameName << " frame.";
            return false;
        }

        if (!m_frameTransformInterface->getTransform(
                m_leftHandFrameName, m_rootFrameName, m_oculusRoot_T_lOculus))
        {
            yError() << "[OculusModule::getTransforms] Unable to evaluate the "
                     << m_leftHandFrameName << " to " << m_rootFrameName << "transformation";
            return false;
        }

        if (!m_frameTransformInterface->getTransform(
                m_rightHandFrameName, m_rootFrameName, m_oculusRoot_T_rOculus))
        {
            yError() << "[OculusModule::getTransforms] Unable to evaluate the "
                     << m_rightHandFrameName << " to " << m_rootFrameName << "transformation";
            return false;
        }
    }
    return true;
}

bool OculusModule::getFeedbacks()
{

    if (m_useXsens)
    {
        if (!m_torso->controlHelper()->getFeedback())
        {
            yError() << "[OculusModule::getFeedbacks] Unable to get the joint encoders feedback: "
                        "TorsoRetargeting";
            return false;
        }
        m_torso->controlHelper()->updateTimeStamp();
    }

    if (!m_head->controlHelper()->getFeedback())
    {
        yError() << "[OculusModule::getFeedbacks] Unable to get the joint encoders feedback: "
                    "HeadRetargeting";
        return false;
    }
    m_head->controlHelper()->updateTimeStamp();

    return true;
}

bool OculusModule::updateModule()
{
    if (!getFeedbacks())
    {
        yError() << "[OculusModule::updateModule] Unable to get the feedback";
        return false;
    }

    if (m_state == OculusFSM::Running)
    {
        // get the transformation form the oculus
        if (!getTransforms())
        {
            yError() << "[OculusModule::updateModule] Unable to get the transform";
            return false;
        }

        if (m_useVirtualizer)
        {
            // in the future the transform server will be used
            yarp::sig::Vector* playerOrientation = m_playerOrientationPort.read(false);
            if (playerOrientation != nullptr)
                m_playerOrientation = (*playerOrientation)(0);

            // used for the image inside the oculus
            yarp::sig::Vector* robotOrientation = m_robotOrientationPort.read(false);
            if (robotOrientation != NULL)
                m_robotYaw = Angles::normalizeAngle((*robotOrientation)(0));
        }

        if (!m_useXsens)
        {
            m_head->setPlayerOrientation(m_playerOrientation);
            m_head->setDesiredHeadOrientation(m_oculusRoot_T_headOculus);
            // m_head->setDesiredHeadOrientation(desiredHeadOrientationVector(0),
            // desiredHeadOrientationVector(1), desiredHeadOrientationVector(2));
            if (!m_head->move())
            {
                yError() << "[OculusModule::updateModule] unable to move the head";
                return false;
            }

            // left hand
            yarp::sig::Vector& leftHandPose = m_leftHandPosePort.prepare();
            m_leftHand->setPlayerOrientation(m_playerOrientation);
            m_leftHand->setHandTransform(m_oculusRoot_T_lOculus);
            m_leftHand->evaluateDesiredHandPose(leftHandPose);
            m_leftHandPosePort.write();

            // right hand
            yarp::sig::Vector& rightHandPose = m_rightHandPosePort.prepare();
            m_rightHand->setPlayerOrientation(m_playerOrientation);
            m_rightHand->setHandTransform(m_oculusRoot_T_rOculus);
            m_rightHand->evaluateDesiredHandPose(rightHandPose);
            m_rightHandPosePort.write();
        }

        // use joypad
        if (!m_useVirtualizer)
        {
            yarp::os::Bottle cmd, outcome;
            double x, y;
            m_joypadControllerInterface->getAxis(m_xJoypadIndex, x);
            m_joypadControllerInterface->getAxis(m_yJoypadIndex, y);

            x = -m_scaleX * deadzone(x);
            y = m_scaleY * deadzone(y);
            std::swap(x, y);

            cmd.addString("setGoal");
            cmd.addDouble(x);
            cmd.addDouble(y);
            m_rpcWalkingClient.write(cmd, outcome);
        }

        // left fingers
        double leftFingersVelocity
            = evaluateDesiredFingersVelocity(m_squeezeLeftIndex, m_releaseLeftIndex);
        if (!m_leftHandFingers->setFingersVelocity(leftFingersVelocity))
        {
            yError() << "[OculusModule::updateModule] Unable to set the left finger velocity.";
            return false;
        }
        if (!m_leftHandFingers->move())
        {
            yError() << "[OculusModule::updateModule] Unable to move the left finger";
            return false;
        }

        // right fingers
        double rightFingersVelocity
            = evaluateDesiredFingersVelocity(m_squeezeRightIndex, m_releaseRightIndex);
        if (!m_rightHandFingers->setFingersVelocity(rightFingersVelocity))
        {
            yError() << "[OculusModule::updateModule] Unable to set the right finger velocity.";
            return false;
        }
        if (!m_rightHandFingers->move())
        {
            yError() << "[OculusModule::updateModule] Unable to move the right finger";
            return false;
        }

    } else if (m_state == OculusFSM::Configured)
    {
        // check if it is time to prepare or start walking
        std::vector<float> buttonMapping(2);

        // prepare robot (A button)
        m_joypadControllerInterface->getButton(m_prepareWalkingIndex, buttonMapping[0]);

        // start walking (X button)
        m_joypadControllerInterface->getButton(m_startWalkingIndex, buttonMapping[1]);

        yarp::os::Bottle cmd, outcome;
        if (buttonMapping[0] > 0)
        {
            // TODO add a visual feedback for the user
            cmd.addString("prepareRobot");
            m_rpcWalkingClient.write(cmd, outcome);
        } else if (buttonMapping[1] > 0)
        {
            if (m_useVirtualizer)
            {
                // reset the player orientation of the virtualizer
                cmd.addString("resetPlayerOrientation");
                m_rpcVirtualizerClient.write(cmd, outcome);
                cmd.clear();
            }

            // TODO add a visual feedback for the user
            cmd.addString("startWalking");
            m_rpcWalkingClient.write(cmd, outcome);
            // if(outcome.get(0).asBool())
            m_state = OculusFSM::Running;
        }
    }
    yarp::os::Bottle& imagesOrientation = m_imagesOrientationPort.prepare();
    imagesOrientation.clear();

    double neckPitch, neckRoll, neckYaw;
    yarp::sig::Vector neckEncoders = m_head->controlHelper()->jointEncoders();
    neckPitch = neckEncoders(0);
    neckRoll = neckEncoders(1);
    neckYaw = neckEncoders(2);

    iDynTree::Rotation chest_R_head
        = HeadRetargeting::forwardKinematics(neckPitch, neckRoll, neckYaw);

    iDynTree::Rotation root_R_chest = iDynTree::Rotation::Identity();
    if (m_useXsens)
    {
        double torsoPitch, torsoRoll, torsoYaw;
        yarp::sig::Vector torsoEncoders = m_torso->controlHelper()->jointEncoders();
        torsoPitch = torsoEncoders(0);
        torsoRoll = torsoEncoders(1);
        torsoYaw = torsoEncoders(2);
        root_R_chest = TorsoRetargeting::forwardKinematics(torsoPitch, torsoRoll, torsoYaw);
    }
    iDynTree::Rotation inertial_R_root = iDynTree::Rotation::RotZ(-m_playerOrientation);

    // inertial_R_head is used to simulate an imu required by the cam calibration application
    iDynTree::Rotation inertial_R_head = inertial_R_root * root_R_chest * chest_R_head;
    iDynTree::Vector3 inertial_R_headRPY = inertial_R_head.asRPY();

    imagesOrientation.addDouble(iDynTree::rad2deg(inertial_R_headRPY(0)));
    imagesOrientation.addDouble(iDynTree::rad2deg(inertial_R_headRPY(1)));
    imagesOrientation.addDouble(iDynTree::rad2deg(inertial_R_headRPY(2)));

    // fake imu
    for (int i = 3; i < 12; i++)
        imagesOrientation.addDouble(0);

    m_imagesOrientationPort.setEnvelope(m_head->controlHelper()->timeStamp());
    m_imagesOrientationPort.write();

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
