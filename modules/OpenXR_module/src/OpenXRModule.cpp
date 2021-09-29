/**
 * @file OpenXRModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

// YARP
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <OpenXRModule.hpp>
#include <Utils.hpp>

#include <functional>

Eigen::Ref<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> getRotation(yarp::sig::Matrix& m)
{
    return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data()).topLeftCorner<3, 3>();
}

auto getPosition(yarp::sig::Matrix& m)
{
    return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data())
        .topRightCorner<3, 1>();
}

auto getPosition(const yarp::sig::Matrix& m)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data())
        .topRightCorner<3, 1>();
}

yarp::sig::Matrix identitySE3()
{
    yarp::sig::Matrix m;
    m.resize(4, 4);
    getRotation(m).setIdentity();
    getPosition(m).setZero();
    m(3, 3) = 1;
    return m;
}

struct OpenXRModule::Impl
{
    struct JoypadParameters
    {
        double deadzone; /**< Joypad deadzone */
        double fullscale; /**< Joypad fullscale */
        double scaleX; /**< Scaling factor on the x axis */
        double scaleY; /**< Scaling factor on the y axis */
        double x; /**< x value */
        double y; /**< y value */
    };
    JoypadParameters joypadParameters;

    enum class OpenXRFSM
    {
        Configured,
        Running,
        InPreparation
    };
    OpenXRFSM state; /**< State of the OpenXRFSM */

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> neckJointsPreparationSmoother{nullptr};
    double preparationSmoothingTime;
    double dT;
    unsigned actuatedDOFs;
    double playerOrientationThreshold;

    std::unique_ptr<HeadRetargeting> head; /**< Pointer to the head retargeting object. */

    // transform server
    yarp::dev::PolyDriver transformClientDevice; /**< Transform client. */
    yarp::dev::IFrameTransform* frameTransformInterface{nullptr}; /**< Frame transform
                                                                       interface. */
    yarp::dev::PolyDriver joypadDevice; /**< Joypad polydriver. */
    yarp::dev::IJoypadController* joypadControllerInterface{nullptr}; /**< joypad interface. */

    std::string rootFrameName; /**< Name of the root frame used in the transform server */
    std::string headFrameName; /**< Name of the head frame used in the transform server (NOT
                                    SUPPORTED BY YARP)*/
    std::string leftHandFrameName; /**< Name of the left hand frame used in the transform server */
    std::string rightHandFrameName; /**< Name of the right hand
                                       frame used in the transform server */

    yarp::sig::Matrix openXRRoot_T_lOpenXR;
    yarp::sig::Matrix openXRRoot_T_rOpenXR;
    yarp::sig::Matrix openXRRoot_T_headOpenXR;
    yarp::sig::Matrix openXRInitialAlignement;

    std::vector<double> openXRHeadsetPoseInertial;

    bool moveRobot{false};
    double playerOrientation{0}; /**< Player orientation (read by the Virtualizer)
                                     only yaw. */

    void initializeNeckJointsSmoother(const unsigned actuatedDOFs,
                                      const double dT,
                                      const double smoothingTime,
                                      const yarp::sig::Vector jointsInitialValue)
    {
        this->neckJointsPreparationSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(
            this->actuatedDOFs, this->dT, smoothingTime);
        this->neckJointsPreparationSmoother->init(jointsInitialValue);
    }

    void getNeckJointsRefSmoothedValues(yarp::sig::Vector& smoothedJointValues)
    {
        yarp::sig::Vector jointValues = {0.0, 0.0, 0.0};
        this->neckJointsPreparationSmoother->computeNextValues(jointValues);
        smoothedJointValues = this->neckJointsPreparationSmoother->getPos();
    }

    bool getFeedbacks()
    {
        if (!this->head->controlHelper()->getFeedback())
        {
            yError() << "[OpenXRModule::getFeedbacks] Unable to get the joint encoders feedback: "
                        "HeadRetargeting";
            return false;
        }
        this->head->controlHelper()->updateTimeStamp();

        return true;
    }

    bool getTransforms()
    {
        // check if everything is ok
        if (!this->frameTransformInterface->frameExists(this->rootFrameName))
        {
            yError() << "[OpenXRModule::getTransforms] No " << this->rootFrameName << " frame.";
            return false;
        }

        if (!this->frameTransformInterface->frameExists(this->headFrameName))
        {
            yError() << "[OpenXRModule::getTransforms] If OpenXr is used the head transform "
                        "must be provided by the transform server.";
            return false;
        }

        if (!this->frameTransformInterface->getTransform(this->headFrameName, //
                                                         this->rootFrameName,
                                                         this->openXRRoot_T_headOpenXR))
        {
            yError() << "[OpenXRModule::getTransforms] Unable to evaluate the "
                     << this->headFrameName << " to " << this->rootFrameName << "transformation";
            return false;
        }

        // This is to remove any initial misplacement and rotations around gravity
        iDynTree::toEigen(this->openXRRoot_T_headOpenXR)
            = iDynTree::toEigen(this->openXRInitialAlignement)
              * iDynTree::toEigen(this->openXRRoot_T_headOpenXR);

        iDynTree::Rotation temp;
        iDynTree::toEigen(temp)
            = iDynTree::toEigen(this->openXRRoot_T_headOpenXR).topLeftCorner<3, 3>();

        temp.getRPY(this->openXRHeadsetPoseInertial[3],
                    this->openXRHeadsetPoseInertial[4],
                    this->openXRHeadsetPoseInertial[5]);
        // this->openXRHeadsetPoseInertial is a 6d std::vector here I'm getting the first three
        // elements
        Eigen::Map<Eigen::Vector3d>(this->openXRHeadsetPoseInertial.data())
            = getPosition(this->openXRRoot_T_headOpenXR);

        // if (!m_useXsens && !m_useIFeel)
        // {

        //     if (!m_frameTransformInterface->frameExists(m_leftHandFrameName))
        //     {

        //         yError() << "[OpenXRModule::getTransforms] No " << m_leftHandFrameName << "
        //         frame."; return false;
        //     }

        //     if (!m_frameTransformInterface->frameExists(m_rightHandFrameName))
        //     {
        //         yError() << "[OpenXRModule::getTransforms] No " << m_rightHandFrameName
        //                  << " frame.";
        //         return false;
        //     }

        //     if (!m_frameTransformInterface->getTransform(
        //             m_leftHandFrameName, m_rootFrameName, m_oculusRoot_T_lOculus))
        //     {
        //         yError() << "[OpenXRModule::getTransforms] Unable to evaluate the "
        //                  << m_leftHandFrameName << " to " << m_rootFrameName << "transformation";
        //         return false;
        //     }

        //     if (!m_frameTransformInterface->getTransform(
        //             m_rightHandFrameName, m_rootFrameName, m_oculusRoot_T_rOculus))
        //     {
        //         yError() << "[OpenXRModule::getTransforms] Unable to evaluate the "
        //                  << m_rightHandFrameName << " to " << m_rootFrameName <<
        //                  "transformation";
        //         return false;
        //     }
        // }
        return true;
    }

    bool configureTranformClient(const yarp::os::Searchable& config,
                                 const std::string& applicationName)
    {
        yarp::os::Property options;
        options.put("device", "transformClient");
        options.put("remote", "/transformServer");
        options.put("local", "/" + applicationName + "/transformClient");

        if (!this->transformClientDevice.open(options))
        {
            yError() << "[OpenXRModule::configureTranformClient] Unable "
                        "to open transformClient device";
            return false;
        }

        // obtain the interface
        if (!this->transformClientDevice.view(this->frameTransformInterface)
            || this->frameTransformInterface == nullptr)
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, "root_frame_name", this->rootFrameName))
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, "head_frame_name", this->headFrameName))
        {

            yError() << "[OpenXRModule::configureTranformClient] Seems that the head "
                        "orientation is not streamed through the transform server.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, //
                                                 "left_hand_frame_name",
                                                 this->leftHandFrameName))
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, //
                                                 "right_hand_frame_name",
                                                 this->rightHandFrameName))
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        this->openXRRoot_T_lOpenXR = identitySE3();
        this->openXRRoot_T_rOpenXR = identitySE3();
        this->openXRRoot_T_headOpenXR = identitySE3();
        this->openXRInitialAlignement = identitySE3();

        return true;
    }

    bool configureJoypad(const yarp::os::Searchable& config, const std::string& name)
    {
        yarp::os::Property options;
        options.put("device", "JoypadControlClient");
        options.put("remote", "/joypadDevice/Oculus");
        options.put("local", "/" + name + "/joypadControlClient");

        if (!this->joypadDevice.open(options))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to open the polydriver.";
            return false;
        }

        // get the interface
        if (!this->joypadDevice.view(this->joypadControllerInterface)
            || this->joypadControllerInterface == nullptr)
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to attach JoypadController"
                        " interface to the PolyDriver object";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config,
                                                 "deadzone", //
                                                 this->joypadParameters.deadzone))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter deadzone";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config, //
                                                 "fullscale",
                                                 this->joypadParameters.fullscale))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter fullscale";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config, "scale_X", this->joypadParameters.scaleX))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter scale_X";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config, "scale_Y", this->joypadParameters.scaleY))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter scale_Y";
            return false;
        }


        // TODO set index of joypad
        // set the index of the axis according to the OVRheadset yarp device
        // bool useLeftStick = config.check("use_left", yarp::os::Value("false")).asBool();
        // m_xJoypadIndex = useLeftStick ? 4 : 6;
        // m_yJoypadIndex = useLeftStick ? 5 : 7;

        // m_squeezeLeftIndex = 0;
        // m_squeezeRightIndex = 1;

        // m_releaseLeftIndex = 2;
        // m_releaseRightIndex = 3;

        // m_startWalkingIndex = 4; // X button
        // m_stopWalkingIndex = 5; // X button
        // m_prepareWalkingIndex = 0; // A button

        return true;
    }

    bool configureOpenXR(const yarp::os::Searchable& config, const std::string& name)
    {
      if (!this->configureTranformClient(config, name))
        {
            yError() << "[OpenXRModule::configureOpenXR] Unable to configure the transform client.";
            return false;
        }

        if (!this->configureJoypad(config, name))
        {
            yError() << "[OpenXRModule::configureOpenXR] Unable to configure the joypad client.";
            return false;
        }

        return true;
    }

    bool resetCamera(const std::string& cameraPort, const std::string& localPort)
    {
        yarp::dev::PolyDriver grabberDriver;

        yarp::os::Property config;
        config.put("device", "remote_grabber");
        config.put("remote", cameraPort);
        config.put("local", localPort);

        bool opened = grabberDriver.open(config);
        if (!opened)
        {
            yError() << "[OpenXRModule::configure] Cannot open remote_grabber device on port "
                     << cameraPort << ".";
            return false;
        }

        yarp::dev::IFrameGrabberControlsDC1394* grabberInterface;

        if (!grabberDriver.view(grabberInterface) || !grabberInterface)
        {
            yError() << "[OpenXRModule::configure] RemoteGrabber does not have "
                        "IFrameGrabberControlDC1394 interface, please update yarp.";
            return false;
        }

        if (!grabberInterface->setResetDC1394())
        {
            yError() << "[OpenXRModule::configure] Failed to reset the camera on port "
                     << cameraPort << ".";
            return false;
        }

        grabberDriver.close();

        return true;
    };
};

OpenXRModule::OpenXRModule()
    : m_pImpl{new Impl()} {};

OpenXRModule::~OpenXRModule(){};



bool OpenXRModule::configure(yarp::os::ResourceFinder& rf)
{
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[OpenXRModule::configure] Empty configuration "
                    "for the OpenXRModule application.";
        return false;
    }

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_pImpl->dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asDouble();
    yInfo() << "[OpenXRModule::configure] sampling time: " << m_pImpl->dT;

    // check if move the robot
    m_pImpl->moveRobot = generalOptions.check("enableMoveRobot", yarp::os::Value(1)).asBool();
    yInfo() << "[OpenXRModule::configure] move the robot: " << m_pImpl->moveRobot;

    // check if move the robot
    m_pImpl->playerOrientationThreshold = generalOptions
                                              .check("playerOrientationThreshold", //
                                                     yarp::os::Value(0.2))
                                              .asDouble();
    yInfo() << "[OpenXRModule::configure] player orientation threshold: "
            << m_pImpl->playerOrientationThreshold;

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    // configure the openxr device
    const std::string headsetGroup = "OPENXR";
    const yarp::os::Bottle& openXROptions = rf.findGroup(headsetGroup);
    if (!m_pImpl->configureOpenXR(openXROptions, getName()))
    {
        yError() << "[OpenXRModule::configure] Unable to configure the oculus";
        return false;
    }

    m_pImpl->head = std::make_unique<HeadRetargeting>();
    yarp::os::Bottle& headOptions = rf.findGroup("HEAD_RETARGETING");
    headOptions.append(generalOptions);
    if (!m_pImpl->head->configure(headOptions, getName()))
    {
        yError() << "[OpenXRModule::configure] Unable to initialize the head retargeting.";
        return false;
    }

    // if (!m_useSenseGlove)
    // {
    //     // configure fingers retargeting
    //     m_leftHandFingers = std::make_unique<FingersRetargeting>();
    //     yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
    //     leftFingersOptions.append(generalOptions);
    //     if (!m_leftHandFingers->configure(leftFingersOptions, getName()))
    //     {
    //         yError()
    //             << "[OpenXRModule::configure] Unable to initialize the left fingers retargeting.";
    //         return false;
    //     }

    //     m_rightHandFingers = std::make_unique<FingersRetargeting>();
    //     yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
    //     rightFingersOptions.append(generalOptions);
    //     if (!m_rightHandFingers->configure(rightFingersOptions, getName()))
    //     {
    //         yError()
    //             << "[OpenXRModule::configure] Unable to initialize the right fingers retargeting.";
    //         return false;
    //     }
    // }

    // configure hands retargeting
    // m_leftHand = std::make_unique<HandRetargeting>();
    // yarp::os::Bottle& leftHandOptions = rf.findGroup("LEFT_HAND_RETARGETING");
    // leftHandOptions.append(generalOptions);
    // if (!m_leftHand->configure(leftHandOptions))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to initialize the left fingers retargeting.";
    //     return false;
    // }

    // m_rightHand = std::make_unique<HandRetargeting>();
    // yarp::os::Bottle& rightHandOptions = rf.findGroup("RIGHT_HAND_RETARGETING");
    // rightHandOptions.append(generalOptions);
    // if (!m_rightHand->configure(rightHandOptions))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to initialize the right fingers retargeting.";
    //     return false;
    // }

    // // open ports
    // std::string portName;
    // if (!YarpHelper::getStringFromSearchable(rf, "leftHandPosePort", portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
    //     return false;
    // }
    // if (!m_leftHandPosePort.open("/" + getName() + portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }

    // if (!YarpHelper::getStringFromSearchable(rf, "rightHandPosePort", portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
    //     return false;
    // }
    // if (!m_rightHandPosePort.open("/" + getName() + portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }

    // if (!m_imagesOrientationPort.open("/" + getName() + "/imagesOrientation:o"))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }
    // if (!m_robotOrientationPort.open("/" + getName() + "/robotOrientation:i"))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }

    // if (!YarpHelper::getStringFromSearchable(rf, "playerOrientationPort", portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
    //     return false;
    // }
    // if (!m_playerOrientationPort.open("/" + getName() + portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }

    // if (!YarpHelper::getStringFromSearchable(rf, "rpcWalkingPort_name", portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
    //     return false;
    // }
    // if (!m_rpcWalkingClient.open("/" + getName() + portName))
    // {
    //     yError() << "[OpenXRModule::configure] " << portName << " port already open.";
    //     return false;
    // }

    // if (!YarpHelper::getStringFromSearchable(rf, "rpcVirtualizerPort_name", portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
    //     return false;
    // }
    // if (!m_rpcVirtualizerClient.open("/" + getName() + portName))
    // {
    //     yError() << "[OpenXRModule::configure] " << portName << " port already open.";
    //     return false;
    // }

    m_pImpl->playerOrientation = 0;
    // m_playerOrientationOld = 0;
    // m_robotYaw = 0;

    m_pImpl->openXRHeadsetPoseInertial.resize(6, 0.0);

    // Reset the cameras if necessary
    bool resetCameras = generalOptions.check("resetCameras", yarp::os::Value(false)).asBool();
    yInfo() << "[OpenXRModule::configure] Reset camera: " << resetCameras;
    if (resetCameras)
    {

        std::string leftCameraPort, rightCameraPort;
        if (!YarpHelper::getStringFromSearchable(generalOptions, "leftCameraPort", leftCameraPort))
        {
            yError() << "[OpenXRModule::configure] resetCameras is true, but leftCameraPort is not "
                        "provided.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(
                generalOptions, "rightCameraPort", rightCameraPort))
        {
            yError() << "[OpenXRModule::configure] resetCameras is true, but rightCameraPort is "
                        "not provided.";
            return false;
        }

        if (!m_pImpl->resetCamera(leftCameraPort, "/walking-teleoperation/camera-reset/left"))
        {
            yError() << "[OpenXRModule::configure] Failed to reset left camera.";
            return false;
        }

        if (!m_pImpl->resetCamera(rightCameraPort, "/walking-teleoperation/camera-reset/right"))
        {
            yError() << "[OpenXRModule::configure] Failed to reset right camera.";
            return false;
        }
        yInfo() << "[OpenXRModule::configure] Cameras have been reset.";
    }

    m_pImpl->state = Impl::OpenXRFSM::Configured;

    return true;
}

double OpenXRModule::getPeriod()
{
    return m_pImpl->dT;
}

bool OpenXRModule::close()
{

    m_pImpl->head->controlHelper()->close();

    // if (!m_useSenseGlove)
    // {
    //     m_rightHandFingers->controlHelper()->close();
    //     m_leftHandFingers->controlHelper()->close();
    // }

    m_pImpl->joypadDevice.close();
    m_pImpl->transformClientDevice.close();

    return true;
}

// double OpenXRModule::evaluateDesiredFingersVelocity(unsigned int squeezeIndex,
//                                                     unsigned int releaseIndex)
// {
//     double releaseFingersVelocity, squeezeFingersVelocity;
//     m_joypadControllerInterface->getAxis(squeezeIndex, squeezeFingersVelocity);
//     m_joypadControllerInterface->getAxis(releaseIndex, releaseFingersVelocity);

//     if (squeezeFingersVelocity > releaseFingersVelocity)
//         return squeezeFingersVelocity;
//     else if (squeezeFingersVelocity < releaseFingersVelocity)
//         return -releaseFingersVelocity;
//     else
//         return 0;
// }

bool OpenXRModule::updateModule()
{
    if (!m_pImpl->getFeedbacks())
    {
        yError() << "[OpenXRModule::updateModule] Unable to get the feedback";
        return false;
    }

    if (m_pImpl->state == Impl::OpenXRFSM::Running)
    {

        // get the transformation form the oculus
        if (!m_pImpl->getTransforms())
        {
            yError() << "[OpenXRModule::updateModule] Unable to get the transform";
            return false;
        }

        m_pImpl->head->setPlayerOrientation(m_pImpl->playerOrientation);
        m_pImpl->head->setDesiredHeadOrientationFromOpenXr(m_pImpl->openXRRoot_T_headOpenXR);

        // m_head->setDesiredHeadOrientation(desiredHeadOrientationVector(0),
        // desiredHeadOrientationVector(1), desiredHeadOrientationVector(2));
        if (m_pImpl->moveRobot)
        {
            if (!m_pImpl->head->move())
            {
                yError() << "[updateModule::updateModule] unable to move the head";
                return false;
            }
        }
        // if (!m_useXsens && !m_useIFeel)
        // {
        //     // update left hand transformation values
        //     yarp::sig::Vector& leftHandPose = m_leftHandPosePort.prepare();
        //     m_leftHand->setPlayerOrientation(m_playerOrientation);
        //     m_leftHand->setHandTransform(m_oculusRoot_T_lOculus);

        //     // update right hand transformation values
        //     yarp::sig::Vector& rightHandPose = m_rightHandPosePort.prepare();
        //     m_rightHand->setPlayerOrientation(m_playerOrientation);
        //     m_rightHand->setHandTransform(m_oculusRoot_T_rOculus);

        //     if (m_useVirtualizer)
        //     {
        //         if (std::abs(m_playerOrientation - m_playerOrientationOld)
        //             > m_playerOrientationThreshold)
        //         {
        //             iDynTree::Position teleopPosition = {m_oculusHeadsetPoseInertial[0],
        //                                                  m_oculusHeadsetPoseInertial[1],
        //                                                  m_oculusHeadsetPoseInertial[2]};

        //             m_leftHand->setPlayerPosition(teleopPosition);
        //             m_rightHand->setPlayerPosition(teleopPosition);
        //             m_playerOrientationOld = m_playerOrientation;
        //         }
        //     }

        //     // evaluate the robot hands' pose
        //     m_leftHand->evaluateDesiredHandPose(leftHandPose);
        //     m_rightHand->evaluateDesiredHandPose(rightHandPose);

        //     // move the robot
        //     if (m_moveRobot)
        //     {
        //         m_leftHandPosePort.write();
        //         m_rightHandPosePort.write();
        //     }
        // }

        // use joypad
        // std::vector<double> locCmd;
        // if (!m_useVirtualizer)
        // {
        //     yarp::os::Bottle cmd, outcome;
        //     double x, y;
        //     m_joypadControllerInterface->getAxis(m_xJoypadIndex, x);
        //     m_joypadControllerInterface->getAxis(m_yJoypadIndex, y);

        //     x = -m_scaleX * deadzone(x);
        //     y = m_scaleY * deadzone(y);
        //     std::swap(x, y);

        //     cmd.addString("setGoal");
        //     cmd.addDouble(x);
        //     cmd.addDouble(y);
        //     if (m_moveRobot)
        //     {
        //         m_rpcWalkingClient.write(cmd, outcome);
        //     }
        //     locCmd.push_back(x);
        //     locCmd.push_back(y);
        // }

        // if (!m_useSenseGlove)
        // {
        //     // left fingers
        //     double leftFingersVelocity
        //         = evaluateDesiredFingersVelocity(m_squeezeLeftIndex, m_releaseLeftIndex);
        //     if (!m_leftHandFingers->setFingersVelocity(leftFingersVelocity))
        //     {
        //         yError() << "[OpenXRModule::updateModule] Unable to set the left finger
        //         velocity."; return false;
        //     }
        //     if (m_moveRobot)
        //     {
        //         if (!m_leftHandFingers->move())
        //         {
        //             yError() << "[OpenXRModule::updateModule] Unable to move the left finger";
        //             return false;
        //         }
        //     }

        //     // right fingers
        //     double rightFingersVelocity
        //         = evaluateDesiredFingersVelocity(m_squeezeRightIndex, m_releaseRightIndex);
        //     if (!m_rightHandFingers->setFingersVelocity(rightFingersVelocity))
        //     {
        //         yError() << "[OpenXRModule::updateModule] Unable to set the right finger
        //         velocity."; return false;
        //     }
        //     if (m_moveRobot)
        //     {
        //         if (!m_rightHandFingers->move())
        //         {
        //             yError() << "[OpenXRModule::updateModule] Unable to move the right finger";
        //             return false;
        //         }
        //     }
        // }

        // // check if it is time to prepare or start walking
        // float buttonMapping;

        // // prepare robot (A button)
        // m_joypadControllerInterface->getButton(m_stopWalkingIndex, buttonMapping);
        // yarp::os::Bottle cmd, outcome;

        // if (buttonMapping > 0)
        // {
        //     // TODO add a visual feedback for the user
        //     if (m_moveRobot)
        //     {
        //         cmd.addString("stopWalking");
        //         m_rpcWalkingClient.write(cmd, outcome);
        //     }
        //     yInfo() << "[OpenXRModule::updateModule] stop";
        //     return false;
        // }
    } else if (m_pImpl->state == Impl::OpenXRFSM::Configured)
    {
        // // check if it is time to prepare or start walking
        // float buttonMapping;

        // // prepare robot (A button)
        // m_joypadControllerInterface->getButton(m_prepareWalkingIndex, buttonMapping);
        // yarp::os::Bottle cmd, outcome;

        // if (buttonMapping > 0)
        // {
        //     // TODO add a visual feedback for the user
        //     if (m_moveRobot)
        //     {
        //         cmd.addString("prepareRobot");
        //         m_rpcWalkingClient.write(cmd, outcome);
        //     }
        //     m_state = OculusFSM::InPreparation;
        //     yInfo() << "[OpenXRModule::updateModule] prepare the robot";
        // }
    } else if (m_pImpl->state == Impl::OpenXRFSM::InPreparation)
    {
        if (m_pImpl->moveRobot)
        {
            m_pImpl->head->initializeNeckJointValues();
            if (!m_pImpl->head->move())
            {
                yError() << "[OpenXRModule::updateModule] unable to move the head";
                return false;
            }
        }

        // TODO reenable  the initial pose resetting
        // float buttonMapping;
        // start walking (X button)


        // m_joypadControllerInterface->getButton(m_startWalkingIndex, buttonMapping);
        // yarp::os::Bottle cmd, outcome;

        // if (buttonMapping > 0)
        // {
        //     if (m_useOpenXr)
        //     {
        //         if (!m_frameTransformInterface->frameExists(m_headFrameName))
        //         {
        //             yError() << "[OpenXRModule::updateModule] The frame named " << m_headFrameName
        //                      << " does not exist.";
        //             yError() << "[OpenXRModule::updateModule] I will not start the walking. Please "
        //                         "try to start again.";
        //             return true;
        //         }

        //         yarp::sig::Matrix openXrHeadInitialTransform = identitySE3();
        //         if (!m_frameTransformInterface->getTransform(
        //                 m_headFrameName, m_rootFrameName, openXrHeadInitialTransform))
        //         {
        //             yError() << "[OpenXRModule::updateModule] Unable to evaluate the "
        //                      << m_headFrameName << " to " << m_rootFrameName << "transformation";
        //             yError() << "[OpenXRModule::updateModule] I will not start the walking. Please "
        //                         "try to start again.";
        //             return true;
        //         }

        //         // get only the yaw axis
        //         iDynTree::Rotation tempRot;
        //         iDynTree::toEigen(tempRot) = getRotation(openXrHeadInitialTransform);
        //         double yaw = 0;
        //         double dummy = 0;
        //         tempRot.getRPY(dummy, yaw, dummy);

        //         iDynTree::Transform tempTransform;
        //         tempTransform.setRotation(iDynTree::Rotation::RotY(
        //             yaw)); // We remove only the initial rotation of the person head around gravity.
        //         tempTransform.setPosition(iDynTree::make_span(getPosition(
        //             openXrHeadInitialTransform))); // We remove the initial position between the
        //                                            // head and the reference frame.

        //         iDynTree::toEigen(m_openXrInitialAlignement)
        //             = iDynTree::toEigen(tempTransform.inverse().asHomogeneousTransform());
        //     }

        //     if (m_useVirtualizer)
        //     {
        //         // not sure if here causes the problem of hand rotation, check it
        //         // reset the player orientation of the virtualizer
        //         cmd.addString("resetPlayerOrientation");
        //         m_rpcVirtualizerClient.write(cmd, outcome);
        //         cmd.clear();
        //     }

        //     // TODO add a visual feedback for the user
        //     if (m_moveRobot)
        //     {
        //         cmd.addString("startWalking");
        //         m_rpcWalkingClient.write(cmd, outcome);
        //     }

        //     // if(outcome.get(0).asBool())
        //     m_state = OculusFSM::Running;
        //     yInfo() << "[OpenXRModule::updateModule] start the robot";
        //     yInfo() << "[OpenXRModule::updateModule] Running ...";
        // }
    }

    return true;
}

// double OpenXRModule::deadzone(const double& input)
// {
//     if (input >= 0)
//     {
//         if (input > m_deadzone)
//             return (input - m_deadzone) / (m_fullscale - m_deadzone);
//         else
//             return 0.0;
//     } else
//     {
//         if (input < -m_deadzone)
//             return (input + m_deadzone) / (m_fullscale - m_deadzone);
//         else
//             return 0.0;
//     }
// }
