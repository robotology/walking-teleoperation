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
#include <yarp/dev/FrameGrabberInterfaces.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <OculusModule.hpp>
#include <Utils.hpp>

#include <functional>

struct OculusModule::Impl
{
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_NeckJointsPreparationSmoother{nullptr};
    double m_PreparationSmoothingTime;
    double m_dT;
    unsigned m_actuatedDOFs;
    void initializeNeckJointsSmoother(const unsigned actuatedDOFs,
                                      const double dT,
                                      const double smoothingTime,
                                      const yarp::sig::Vector jointsInitialValue);
    void getNeckJointsRefSmoothedValues(yarp::sig::Vector& smoothedJointValues);
};

OculusModule::OculusModule()
    : pImpl{new Impl()} {};

OculusModule::~OculusModule(){};

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
        // oculus orientation values
        std::string portName;
        if (!YarpHelper::getStringFromSearchable(config, "oculusOrientationPort", portName))
        {
            yError() << "[OculusModule::configureTranformClient] Unable to get string "
                        "(oculusOrientationPort) from a "
                        "searchable";
            return false;
        }

        if (!m_oculusOrientationPort.open("/" + getName() + portName))
        {
            yError() << "[OculusModule::configureTranformClient] Unable to open the port oculus "
                        "orientation "
                     << "/" << getName() << portName;
            return false;
        }

        // oculus position values
        if (!YarpHelper::getStringFromSearchable(config, "oculusPositionPort", portName))
        {
            yError() << "[OculusModule::configureTranformClient] Unable to get a string "
                        "(oculusPositionPort) from a "
                        "searchable";
            return false;
        }

        if (!m_oculusPositionPort.open("/" + getName() + portName))
        {
            yError()
                << "[OculusModule::configureTranformClient] Unable to open the port oculus position"
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
    m_stopWalkingIndex = 5; // X button
    m_prepareWalkingIndex = 0; // A button

    return true;
}

bool OculusModule::resetCamera(const std::string &cameraPort, const std::string &localPort)
{
    yarp::dev::PolyDriver grabberDriver;

    yarp::os::Property config;
    config.put("device", "remote_grabber");
    config.put("remote", cameraPort);
    config.put("local", localPort);

    bool opened = grabberDriver.open(config);
    if(!opened)
    {
        yError() << "[OculusModule::configure] Cannot open remote_grabber device on port " << cameraPort <<".";
        return false;
    }

    yarp::dev::IFrameGrabberControlsDC1394 *grabberInterface;

    if(!grabberDriver.view(grabberInterface) || !grabberInterface)
    {
        yError() << "[OculusModule::configure] RemoteGrabber does not have IFrameGrabberControlDC1394 interface, please update yarp.";
        return false;
    }

    if(!grabberInterface->setResetDC1394())
    {
        yError() << "[OculusModule::configure] Failed to reset the camera on port " << cameraPort << ".";
        return false;
    }

    grabberDriver.close();

    return true;
};

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
#ifdef ENABLE_LOGGER
    yInfo() << "[OculusModule::configure] matlogger2 is eanbled!";
#else
    yInfo() << "[OculusModule::configure] matlogger2 is disabled!";
#endif

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

    // check if move the robot
    m_moveRobot = generalOptions.check("enableMoveRobot", yarp::os::Value(1)).asBool();
    yInfo() << "[OculusModule::configure] move the robot: " << m_moveRobot;

    // check if move the robot
    m_playerOrientationThreshold
        = generalOptions.check("playerOrientationThreshold", yarp::os::Value(0.2)).asDouble();
    yInfo() << "[OculusModule::configure] player orientation threshold: "
            << m_playerOrientationThreshold;

    // check if log the data
    m_enableLogger = generalOptions.check("enableLogger", yarp::os::Value(0)).asBool();

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

    m_useSenseGlove = generalOptions.check("useSenseGlove", yarp::os::Value(false)).asBool();
    yInfo() << "Teleoperation uses SenseGlove: " << m_useSenseGlove;

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

    if (!m_useSenseGlove)
    {
        // configure fingers retargeting
        m_leftHandFingers = std::make_unique<FingersRetargeting>();
        yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
        leftFingersOptions.append(generalOptions);
        if (!m_leftHandFingers->configure(leftFingersOptions, getName()))
        {
            yError()
                << "[OculusModule::configure] Unable to initialize the left fingers retargeting.";
            return false;
        }

        m_rightHandFingers = std::make_unique<FingersRetargeting>();
        yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
        rightFingersOptions.append(generalOptions);
        if (!m_rightHandFingers->configure(rightFingersOptions, getName()))
        {
            yError()
                << "[OculusModule::configure] Unable to initialize the right fingers retargeting.";
            return false;
        }
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
    m_playerOrientationOld = 0;
    m_robotYaw = 0;

    // open the logger only if all the vecotos sizes are clear.
    if (m_enableLogger)
    {
        if (!openLogger())
        {
            yError() << "[OculusModule::configure] Unable to open the logger";
            return false;
        }
    }
    m_oculusHeadsetPoseInertial.resize(6, 0.0);

    //Reset the cameras if necessary
    bool resetCameras = generalOptions.check("resetCameras", yarp::os::Value(false)).asBool();
    yInfo() << "[OculusModule::configure] Reset camera: " << resetCameras;
    if (resetCameras)
    {

        std::string leftCameraPort, rightCameraPort;
        if (!YarpHelper::getStringFromSearchable(generalOptions, "leftCameraPort", leftCameraPort))
        {
            yError() << "[OculusModule::configure] resetCameras is true, but leftCameraPort is not provided.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(generalOptions, "rightCameraPort", rightCameraPort))
        {
            yError() << "[OculusModule::configure] resetCameras is true, but rightCameraPort is not provided.";
            return false;
        }

        if (!resetCamera(leftCameraPort,"/walking-teleoperation/camera-reset/left"))
        {
            yError() << "[OculusModule::configure] Failed to reset left camera.";
            return false;
        }

        if (!resetCamera(rightCameraPort,"/walking-teleoperation/camera-reset/right"))
        {
            yError() << "[OculusModule::configure] Failed to reset right camera.";
            return false;
        }
        yInfo() << "[OculusModule::configure] Cameras have been reset.";
    }

    m_state = OculusFSM::Configured;

    return true;
}

double OculusModule::getPeriod()
{
    return m_dT;
}

bool OculusModule::close()
{
#ifdef ENABLE_LOGGER
    if (m_enableLogger)
    {
        m_logger->flush_available_data();
    }
#endif
    // m_logger.reset();

    // close devices
    m_head->controlHelper()->close();

    if (!m_useSenseGlove)
    {
        m_rightHandFingers->controlHelper()->close();
        m_leftHandFingers->controlHelper()->close();
    }

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
            // get head orientation
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

                m_oculusHeadsetPoseInertial[3] = -desiredHeadOrientationVector(1);
                m_oculusHeadsetPoseInertial[4] = desiredHeadOrientationVector(0);
                m_oculusHeadsetPoseInertial[5] = desiredHeadOrientationVector(2);
            }

            // get head position
            yarp::os::Bottle* desiredHeadPosition = NULL;
            iDynTree::Vector3 desiredHeadPositionVector;
            desiredHeadPosition = m_oculusPositionPort.read(false);
            if (desiredHeadPosition != NULL)
            {
                for (unsigned i = 0; i < desiredHeadPosition->size(); i++)
                {
                    desiredHeadPositionVector(i) = desiredHeadPosition->get(i).asDouble();
                }

                // the data coming from oculus vr is with the following order:
                // [x,y,z]
                // coordinate system definition is provided in:
                // https://developer.oculus.com/documentation/pcsdk/latest/concepts/dg-sensor/
                //            iDynTree::toEigen(m_oculusRoot_T_headOculus).block(0, 3, 3, 1)
                //                = iDynTree::toEigen(desiredHeadPositionVector);

                m_oculusHeadsetPoseInertial[0] = desiredHeadPositionVector(0);
                m_oculusHeadsetPoseInertial[1] = desiredHeadPositionVector(1);
                m_oculusHeadsetPoseInertial[2] = desiredHeadPositionVector(2);
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
            m_head->evalueNeckJointValues();
            // m_head->setDesiredHeadOrientation(desiredHeadOrientationVector(0),
            // desiredHeadOrientationVector(1), desiredHeadOrientationVector(2));
            if (m_moveRobot)
            {
                if (!m_head->move())
                {
                    yError() << "[updateModule::updateModule] unable to move the head";
                    return false;
                }
            }
            // update left hand transformation values
            yarp::sig::Vector& leftHandPose = m_leftHandPosePort.prepare();
            m_leftHand->setPlayerOrientation(m_playerOrientation);
            m_leftHand->setHandTransform(m_oculusRoot_T_lOculus);

            // update right hand transformation values
            yarp::sig::Vector& rightHandPose = m_rightHandPosePort.prepare();
            m_rightHand->setPlayerOrientation(m_playerOrientation);
            m_rightHand->setHandTransform(m_oculusRoot_T_rOculus);

            if (m_useVirtualizer)
            {
                if (std::abs(m_playerOrientation - m_playerOrientationOld)
                    > m_playerOrientationThreshold)
                {
                    iDynTree::Position teleopPosition = {m_oculusHeadsetPoseInertial[0],
                                                         m_oculusHeadsetPoseInertial[1],
                                                         m_oculusHeadsetPoseInertial[2]};

                    m_leftHand->setPlayerPosition(teleopPosition);
                    m_rightHand->setPlayerPosition(teleopPosition);
                    m_playerOrientationOld = m_playerOrientation;
                }
            }

            // evaluate the robot hands' pose
            m_leftHand->evaluateDesiredHandPose(leftHandPose);
            m_rightHand->evaluateDesiredHandPose(rightHandPose);

            // move the robot
            if (m_moveRobot)
            {
                m_leftHandPosePort.write();
                m_rightHandPosePort.write();
            }
        }

        // use joypad
        std::vector<double> locCmd;
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
            if (m_moveRobot)
            {
                m_rpcWalkingClient.write(cmd, outcome);
            }
            locCmd.push_back(x);
            locCmd.push_back(y);
        }

        if (!m_useSenseGlove)
        {
            // left fingers
            double leftFingersVelocity
                = evaluateDesiredFingersVelocity(m_squeezeLeftIndex, m_releaseLeftIndex);
            if (!m_leftHandFingers->setFingersVelocity(leftFingersVelocity))
            {
                yError() << "[OculusModule::updateModule] Unable to set the left finger velocity.";
                return false;
            }
            if (m_moveRobot)
            {
                if (!m_leftHandFingers->move())
                {
                    yError() << "[OculusModule::updateModule] Unable to move the left finger";
                    return false;
                }
            }

            // right fingers
            double rightFingersVelocity
                = evaluateDesiredFingersVelocity(m_squeezeRightIndex, m_releaseRightIndex);
            if (!m_rightHandFingers->setFingersVelocity(rightFingersVelocity))
            {
                yError() << "[OculusModule::updateModule] Unable to set the right finger velocity.";
                return false;
            }
            if (m_moveRobot)
            {
                if (!m_rightHandFingers->move())
                {
                    yError() << "[OculusModule::updateModule] Unable to move the right finger";
                    return false;
                }
            }
        }

        // check if it is time to prepare or start walking
        float buttonMapping;

        // prepare robot (A button)
        m_joypadControllerInterface->getButton(m_stopWalkingIndex, buttonMapping);
        yarp::os::Bottle cmd, outcome;

        if (buttonMapping > 0)
        {
            // TODO add a visual feedback for the user
            if (m_moveRobot)
            {
                cmd.addString("stopWalking");
                m_rpcWalkingClient.write(cmd, outcome);
            }
            yInfo() << "[OculusModule::updateModule] stop";
            return false;
        }
        
#ifdef ENABLE_LOGGER
        if (m_enableLogger)
        {
            m_logger->add(m_logger_prefix + "_time", yarp::os::Time::now());
            m_logger->add(m_logger_prefix + "_playerOrientation", m_playerOrientation);
            if (m_moveRobot)
            {
                m_logger->add(m_logger_prefix + "_robotYaw", m_robotYaw);
            } else
            {
                m_logger->add(m_logger_prefix + "_robotYaw", 0.0);
            }

            std::vector<double> neckAngles;
            yarp::sig::Vector neckValuesSig;
            m_head->getNeckJointValues(neckValuesSig);
            for (unsigned i = 0; i < neckValuesSig.size(); i++)
            {
                neckAngles.push_back(neckValuesSig(i));
            }
            m_logger->add(m_logger_prefix + "_neckJointValues", neckAngles);

            std::vector<double> lFingers, rFingers;
            m_leftHandFingers->getFingerValues(lFingers);
            m_rightHandFingers->getFingerValues(rFingers);
            if (lFingers.size() > 0)
            {
                m_logger->add(m_logger_prefix + "_leftFingerValues", lFingers);
            }
            if (rFingers.size() > 0)
            {
                m_logger->add(m_logger_prefix + "_rightFingerValues", rFingers);
            }

            std::vector<double> left_robotHandpose_robotTel, left_humanHandpose_oculusInertial,
                left_humanHandpose_humanTel;
            m_leftHand->getHandInfo(left_robotHandpose_robotTel,
                                    left_humanHandpose_oculusInertial,
                                    left_humanHandpose_humanTel);

            m_logger->add(m_logger_prefix + "_left_robotHandpose_robotTeleoperation",
                          left_robotHandpose_robotTel);
            m_logger->add(m_logger_prefix + "_left_humanHandpose_oculusInertial",
                          left_humanHandpose_oculusInertial);
            m_logger->add(m_logger_prefix + "_left_humanHandpose_humanTeleoperation",
                          left_humanHandpose_humanTel);

            std::vector<double> right_robotHandpose_robotTel, right_humanHandpose_oculusInertial,
                right_humanHandpose_humanTel;
            m_rightHand->getHandInfo(right_robotHandpose_robotTel,
                                     right_humanHandpose_oculusInertial,
                                     right_humanHandpose_humanTel);

            m_logger->add(m_logger_prefix + "_right_robotHandpose_robotTeleoperation",
                          right_robotHandpose_robotTel);
            m_logger->add(m_logger_prefix + "_right_humanHandpose_oculusInertial",
                          right_humanHandpose_oculusInertial);
            m_logger->add(m_logger_prefix + "_right_humanHandpose_humanTeleoperation",
                          right_humanHandpose_humanTel);

            m_logger->add(m_logger_prefix + "_oculusHeadset_Inertial",
                          m_oculusHeadsetPoseInertial); // pose sizein 3D space

            if (!m_useVirtualizer)
            {
                m_logger->add(m_logger_prefix + "_loc_joypad_x_y", locCmd);
            }

            m_logger->flush_available_data();
        }
#endif
    } else if (m_state == OculusFSM::Configured)
    {
        // check if it is time to prepare or start walking
        float buttonMapping;

        // prepare robot (A button)
        m_joypadControllerInterface->getButton(m_prepareWalkingIndex, buttonMapping);
        yarp::os::Bottle cmd, outcome;

        if (buttonMapping > 0)
        {
            // TODO add a visual feedback for the user
            if (m_moveRobot)
            {
                cmd.addString("prepareRobot");
                m_rpcWalkingClient.write(cmd, outcome);
            }
            m_state = OculusFSM::InPreparation;
            yInfo() << "[OculusModule::updateModule] prepare the robot";
        }
    } else if (m_state == OculusFSM::InPreparation)
    {
        if (m_moveRobot)
        {
            m_head->initializeNeckJointValues();
            if (!m_head->move())
            {
                yError() << "[updateModule::updateModule] unable to move the head";
                return false;
            }
        }

        float buttonMapping;
        // start walking (X button)
        m_joypadControllerInterface->getButton(m_startWalkingIndex, buttonMapping);
        yarp::os::Bottle cmd, outcome;

        if (buttonMapping > 0)
        {
            if (m_useVirtualizer)
            {
                // not sure if here causes the problem of hand rotation, check it
                // reset the player orientation of the virtualizer
                cmd.addString("resetPlayerOrientation");
                m_rpcVirtualizerClient.write(cmd, outcome);
                cmd.clear();
            }

            // TODO add a visual feedback for the user
            if (m_moveRobot)
            {
                cmd.addString("startWalking");
                m_rpcWalkingClient.write(cmd, outcome);
            }
            // if(outcome.get(0).asBool())
            m_state = OculusFSM::Running;
            yInfo() << "[OculusModule::updateModule] start the robot";
            yInfo() << "[OculusModule::updateModule] Running ...";
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

bool OculusModule::openLogger()
{
#ifdef ENABLE_LOGGER
    std::string currentTime = getTimeDateMatExtension();
    std::string fileName = "OculusModule" + currentTime + "log.mat";

    m_logger = XBot::MatLogger2::MakeLogger(fileName);
    m_appender = XBot::MatAppender::MakeInstance();
    m_appender->add_logger(m_logger);
    m_appender->start_flush_thread();

    m_logger->create(m_logger_prefix + "_time", 1);
    m_logger->create(m_logger_prefix + "_playerOrientation", 1);

    m_logger->create(m_logger_prefix + "_robotYaw", 1);

    m_logger->create(m_logger_prefix + "_neckJointValues", m_head->controlHelper()->getDoFs());
    m_logger->create(m_logger_prefix + "_leftFingerValues",
                     m_leftHandFingers->controlHelper()->getDoFs());
    m_logger->create(m_logger_prefix + "_rightFingerValues",
                     m_rightHandFingers->controlHelper()->getDoFs());

    m_logger->create(m_logger_prefix + "_left_robotHandpose_robotTeleoperation",
                     6); // pose sizein 3D space
    m_logger->create(m_logger_prefix + "_left_humanHandpose_oculusInertial",
                     6); // pose sizein 3D space
    m_logger->create(m_logger_prefix + "_left_humanHandpose_humanTeleoperation",
                     6); // pose sizein 3D space

    m_logger->create(m_logger_prefix + "_right_robotHandpose_robotTeleoperation",
                     6); // pose sizein 3D space
    m_logger->create(m_logger_prefix + "_right_humanHandpose_oculusInertial",
                     6); // pose sizein 3D space
    m_logger->create(m_logger_prefix + "_right_humanHandpose_humanTeleoperation",
                     6); // pose sizein 3D space
    m_logger->create(m_logger_prefix + "_oculusHeadset_Inertial",
                     6); // pose sizein 3D space

    m_logger->create(m_logger_prefix + "_loc_joypad_x_y",
                     2); // [x,y] component for robot locomotion
    yInfo() << "[OculusModule::openLogger] Logging is active.";
#else
    yInfo() << "[OculusModule::openLogger] option is not active in CMakeLists.";

#endif
    return true;
}

void OculusModule::Impl::initializeNeckJointsSmoother(const unsigned m_actuatedDOFs,
                                                      const double m_dT,
                                                      const double smoothingTime,
                                                      const yarp::sig::Vector jointsInitialValue)
{
    m_NeckJointsPreparationSmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_actuatedDOFs, m_dT, smoothingTime);
    m_NeckJointsPreparationSmoother->init(jointsInitialValue);
}
void OculusModule::Impl::getNeckJointsRefSmoothedValues(yarp::sig::Vector& smoothedJointValues)
{
    yarp::sig::Vector jointValues = {0.0, 0.0, 0.0};
    m_NeckJointsPreparationSmoother->computeNextValues(jointValues);
    smoothedJointValues = m_NeckJointsPreparationSmoother->getPos();
}
