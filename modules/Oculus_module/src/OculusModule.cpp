// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// YARP
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <OculusModule.hpp>
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

yarp::sig::Matrix identitySE3()
{
    yarp::sig::Matrix m;
    m.resize(4, 4);
    getRotation(m).setIdentity();
    getPosition(m).setZero();
    m(3, 3) = 1;
    return m;
}

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
    options.put("device", config.check("transform_server_device", yarp::os::Value("frameTransformClient")).asString());
    options.put("filexml_option",  config.check("transform_server_file", yarp::os::Value("ftc_yarp_only.xml")).asString());
    options.put("ft_client_prefix", config.check("transform_server_local", yarp::os::Value(getName() + "/tf")).asString());
    if (config.check("transform_server_remote"))
    {
        options.put("ft_server_prefix", config.find("transform_server_remote").asString());
    }
    options.put("local_rpc", "/" + getName() + "/tf/local_rpc");

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

    m_oculusRoot_T_lOculus = identitySE3();
    m_oculusRoot_T_rOculus = identitySE3();
    m_oculusRoot_T_headOculus = identitySE3();
    m_openXrInitialAlignement = identitySE3();

    return true;
}

bool OculusModule::configureJoypad(const yarp::os::Searchable& config)
{
    m_skipJoypad = config.check("skip_joypad", yarp::os::Value(false)).asBool();
    m_useVirtualizer = !(config.check("move_icub_using_joypad", yarp::os::Value(false)).asBool());

    if (m_skipJoypad && !m_useVirtualizer)
    {
        yError() << "[OculusModule::configureJoypad] skip_joypad and move_icub_using_joypad cannot "
                    "be true at the same time.";
        return false;
    }

    if (!m_useVirtualizer)
    {
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

    yarp::os::Property options;
    options.put("device", "JoypadControlClient");
    options.put("remote", "/joypadDevice/Oculus");
    options.put("local", "/" + getName() + "/joypadControlClient");

    if (!m_skipJoypad)
    {
        if (m_joypadDevice.open(options))
        {
            // get the interface
            if (!m_joypadDevice.view(m_joypadControllerInterface) || !m_joypadControllerInterface)
            {
                if (m_useSenseGlove && m_useVirtualizer)
                {
                    yWarning() << "[OculusModule::configureJoypad] Unable to attach "
                                  "JoypadController interface "
                                  "to the PolyDriver object. Continuing anyway since we are using "
                                  "the virtualizer and the gloves."
                                  "Set skip_joypad to true to avoid this warning.";
                } else
                {
                    yError() << "[OculusModule::configureJoypad] Unable to attach JoypadController "
                                "interface "
                                "to the PolyDriver object";
                    return false;
                }
            }
        } else
        {
            if (m_useSenseGlove && m_useVirtualizer)
            {
                yWarning() << "[OculusModule::configureJoypad] Unable to open the polydriver. "
                              "Continuing anyway since we are using the virtualizer and the gloves."
                              "Set skip_joypad to true to avoid this warning.";
            } else
            {
                yError() << "[OculusModule::configureJoypad] Unable to open the polydriver.";
                return false;
            }
        }
    }

    return true;
}

bool OculusModule::resetCamera(const std::string& cameraPort, const std::string& localPort)
{
    yarp::dev::PolyDriver grabberDriver;

    yarp::os::Property config;
    config.put("device", "remote_grabber");
    config.put("remote", cameraPort);
    config.put("local", localPort);

    bool opened = grabberDriver.open(config);
    if (!opened)
    {
        yError() << "[OculusModule::configure] Cannot open remote_grabber device on port "
                 << cameraPort << ".";
        return false;
    }

    yarp::dev::IFrameGrabberControlsDC1394* grabberInterface;

    if (!grabberDriver.view(grabberInterface) || !grabberInterface)
    {
        yError() << "[OculusModule::configure] RemoteGrabber does not have "
                    "IFrameGrabberControlDC1394 interface, please update yarp.";
        return false;
    }

    if (!grabberInterface->setResetDC1394())
    {
        yError() << "[OculusModule::configure] Failed to reset the camera on port " << cameraPort
                 << ".";
        return false;
    }

    grabberDriver.close();

    return true;
};

bool OculusModule::setCameraAutoMode(const std::string& cameraPort, const std::string& localPort)
{
    yarp::dev::PolyDriver grabberDriver;

    yarp::os::Property config;
    config.put("device", "remote_grabber");
    config.put("remote", cameraPort);
    config.put("local", localPort);

    bool opened = grabberDriver.open(config);
    if (!opened)
    {
        yError() << "[OculusModule::configure] Cannot open remote_grabber device on port "
                 << cameraPort << ".";
        return false;
    }

    yarp::dev::IFrameGrabberControls* grabberInterface;

    if (!grabberDriver.view(grabberInterface) || !grabberInterface)
    {
        yError() << "[OculusModule::configure] RemoteGrabber does not have "
                    "IFrameGrabberControls interface.";
        return false;
    }

    auto setAuto = [grabberInterface, &cameraPort](cameraFeature_id_t feature, const std::string& featureName) -> bool
    {
        bool hasFeature = false;
        bool hasAuto = false;
        grabberInterface->hasFeature(feature, &hasFeature);
        grabberInterface->hasAuto(feature, &hasAuto);

        if (!hasFeature || !hasAuto)
        {
            yWarning() << "[OculusModule::configure] Cannot set AUTO to" << featureName << "on" << cameraPort
                       << ".";
              return true;
        }

        if (!grabberInterface->setActive(feature, true))
        {
            yError() << "[OculusModule::configure] Failed to set AUTO" << featureName << "on"
                     << cameraPort << ".";
            return false;
        }

        if (!grabberInterface->setMode(feature, FeatureMode::MODE_AUTO))
        {
            yError() << "[OculusModule::configure] Failed to set AUTO" << featureName << "on" << cameraPort
                     << ".";
            return false;
        }

        return true;
    };

    std::initializer_list<std::pair<cameraFeature_id_t, std::string>> features =
    {{cameraFeature_id_t::YARP_FEATURE_SHUTTER, "shutter"},
     {cameraFeature_id_t::YARP_FEATURE_BRIGHTNESS, "brightness"},
     {cameraFeature_id_t::YARP_FEATURE_GAIN, "gain"},
     {cameraFeature_id_t::YARP_FEATURE_EXPOSURE, "exposure"},
     {cameraFeature_id_t::YARP_FEATURE_WHITE_BALANCE, "white balance"},
     {cameraFeature_id_t::YARP_FEATURE_SHARPNESS, "sharpness"},
     {cameraFeature_id_t::YARP_FEATURE_SATURATION, "saturation"},
    };

    for (auto& feat : features)
    {
        if (!setAuto(feat.first, feat.second))
        {
            return false;
        }
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
    std::lock_guard<std::mutex> guard(m_mutex);

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
    m_dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asFloat64();

    // check if move the robot
    m_moveRobot = generalOptions.check("enableMoveRobot", yarp::os::Value(1)).asBool();
    yInfo() << "[OculusModule::configure] move the robot: " << m_moveRobot;

    // check if move the robot
    m_playerOrientationThreshold
        = generalOptions.check("playerOrientationThreshold", yarp::os::Value(0.2)).asFloat64();
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

    m_useIFeel = generalOptions.check("useiFeel", yarp::os::Value(false)).asBool();
    yInfo() << "Teleoperation uses iFeel: " << m_useIFeel;

    m_useSenseGlove = generalOptions.check("useSenseGlove", yarp::os::Value(false)).asBool();
    yInfo() << "Teleoperation uses SenseGlove: " << m_useSenseGlove;

    m_useOpenXr = generalOptions.check("useOpenXr", yarp::os::Value(false)).asBool();
    yInfo() << "Teleoperation uses OpenXr: " << m_useOpenXr;

    if (m_useOpenXr && (!m_useIFeel && !m_useXsens))
    {
        yError() << "[OculusModule::configure] You cannot use OpenXr without xsens of iFeel. "
                    "This feature will be implemented in the future.";
        return false;
    }
    std::string headsetGroup = m_useOpenXr ? "OPENXR" : "OCULUS";

    yarp::os::Bottle& oculusOptions = rf.findGroup(headsetGroup);
    if (!configureOculus(oculusOptions))
    {
        yError() << "[OculusModule::configure] Unable to configure the oculus";
        return false;
    }

    // configure head retargeting
    if (!m_useXsens)
    {
        m_head = std::make_unique<HeadRetargeting>();
        yarp::os::Bottle& headOptions = rf.findGroup("HEAD_RETARGETING");
        headOptions.append(generalOptions);
        if (!m_head->configure(headOptions, getName()))
        {
            yError() << "[OculusModule::configure] Unable to initialize the head retargeting.";
            return false;
        }
    }

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

    if (!YarpHelper::getStringFromSearchable(rf, "rpcServerOculusPort_name", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    this->yarp().attachAsServer(this->m_rpcOculusServerPort);

    if (!m_rpcOculusServerPort.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] Cannot open " << portName << " RPC port .";
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

    // Reset the cameras if necessary
    bool resetCameras = generalOptions.check("resetCameras", yarp::os::Value(false)).asBool();
    yInfo() << "[OculusModule::configure] Reset camera: " << resetCameras;

    // Reset the cameras if necessary
    bool setCamerasAutoMode = generalOptions.check("setCamerasAutoMode", yarp::os::Value(false)).asBool();
    yInfo() << "[OculusModule::configure] Set cameras auto mode: " << setCamerasAutoMode;

    if (resetCameras || setCamerasAutoMode)
    {

        std::string leftCameraPort, rightCameraPort;
        if (!YarpHelper::getStringFromSearchable(generalOptions, "leftCameraPort", leftCameraPort))
        {
            yError() << "[OculusModule::configure] resetCameras is true, but leftCameraPort is not "
                        "provided.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(
                generalOptions, "rightCameraPort", rightCameraPort))
        {
            yError() << "[OculusModule::configure] resetCameras is true, but rightCameraPort is "
                        "not provided.";
            return false;
        }

        if (resetCameras)
        {
            if (!resetCamera(leftCameraPort, "/walking-teleoperation/camera-reset/left"))
            {
                yError() << "[OculusModule::configure] Failed to reset left camera.";
                return false;
            }

            if (!resetCamera(rightCameraPort, "/walking-teleoperation/camera-reset/right"))
            {
                yError() << "[OculusModule::configure] Failed to reset right camera.";
                return false;
            }

            yInfo() << "[OculusModule::configure] Cameras have been reset.";
        }

        if (setCamerasAutoMode)
        {
            if (!setCameraAutoMode(leftCameraPort, "/walking-teleoperation/camera-auto/left"))
            {
                yError() << "[OculusModule::configure] Failed to set left camera to AUTO mode.";
                return false;
            }

            if (!setCameraAutoMode(rightCameraPort, "/walking-teleoperation/camera-auto/right"))
            {
                yError() << "[OculusModule::configure] Failed to set right camera to AUTO mode.";
                return false;
            }

            yInfo() << "[OculusModule::configure] Cameras have been set to AUTO.";
        }

    }

    m_state = OculusFSM::Configured;

    m_autostart
        = generalOptions.check("autostart")
                     && (generalOptions.find("autostart").isNull()
                         || generalOptions.find("autostart").asBool()); //True if autostart is set but with no value or if the value is true
    m_autostartDelay = generalOptions.check("autostartDelay", yarp::os::Value(10.0)).asFloat64();

    m_initializationUseFullRotation = generalOptions.check("initializationUseFullRotation", yarp::os::Value(false)).asBool();

    if (m_autostart)
    {
        m_autostartConfigureTime = yarp::os::Time::now();
        this->preparingModule();
    }


    return true;
}

double OculusModule::getPeriod()
{
    return m_dT;
}

bool OculusModule::close()
{
    std::lock_guard<std::mutex> guard(m_mutex);

#ifdef ENABLE_LOGGER
    if (m_enableLogger)
    {
        m_logger->flush_available_data();
    }
#endif
    // m_logger.reset();

    // close devices
    if (!m_useXsens)
    {
        m_head->controlHelper()->close();
    }

    if (!m_useSenseGlove)
    {
        m_rightHandFingers->controlHelper()->close();
        m_leftHandFingers->controlHelper()->close();
    }

    m_joypadDevice.close();
    m_transformClientDevice.close();

    m_rpcOculusServerPort.close();
    m_rpcVirtualizerClient.close();
    m_rpcWalkingClient.close();
    m_oculusPositionPort.close();
    m_oculusOrientationPort.close();
    m_imagesOrientationPort.close();
    m_playerOrientationPort.close();
    m_rightHandPosePort.close();
    m_leftHandPosePort.close();

    return true;
}

std::string OculusModule::getStringFromOculusState(const OculusFSM state)
{
    switch (state)
    {
    case (OculusFSM::Configured):
        return "OculusFSM::Configured";
    case (OculusFSM::InPreparation):
        return "OculusFSM::InPreparation";
    case (OculusFSM::Running):
        return "OculusFSM::Running";
    default:
        return "";
    }
    return "";
}

bool OculusModule::prepareTeleoperation()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_state != OculusFSM::Configured)
    {
        yWarning() << "The teleoperation is in state: " + getStringFromOculusState(m_state)
                          + ", while it should be in state: "
                          + getStringFromOculusState(OculusFSM::Configured)
                   << "in order to prepare the teleoperation.";
        yWarning() << "No update on the teleoperation state, try again.";
        return false;
    }

    return this->preparingModule();
}

bool OculusModule::runTeleoperation()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if (m_state != OculusFSM::InPreparation)
    {
        yWarning() << "The teleoperation is in state: " + getStringFromOculusState(m_state)
                          + ", while it should be in state: "
                          + getStringFromOculusState(OculusFSM::InPreparation)
                   << "in order to run the teleoperation.";
        yWarning() << "No update on the teleoperation state, try again.";
        return false;
    }

    return this->runningModule();
}

bool OculusModule::preparingModule()
{
    yarp::os::Bottle cmd, outcome;
    if (m_moveRobot)
    {
        cmd.addString("prepareRobot");
        m_rpcWalkingClient.write(cmd, outcome);
    }
    m_state = OculusFSM::InPreparation;
    yInfo() << "[OculusModule::preparingModule] prepare the robot";

    return true;
}

bool OculusModule::runningModule()
{
    yarp::os::Bottle cmd, outcome;

    if (m_useOpenXr)
    {
        if (!m_frameTransformInterface->frameExists(m_headFrameName))
        {
            yError() << "[OculusModule::runningModule] The frame named " << m_headFrameName
                     << " does not exist.";
            yError() << "[OculusModule::runningModule] I will not start the walking. Please "
                        "try to start again.";
            return true;
        }
        yarp::sig::Matrix openXrHeadInitialTransform = identitySE3();
        if (!m_frameTransformInterface->getTransform(
                    m_headFrameName, m_rootFrameName, openXrHeadInitialTransform))
        {
            yError() << "[OculusModule::runningModule] Unable to evaluate the " << m_headFrameName
                     << " to " << m_rootFrameName << "transformation";
            yError() << "[OculusModule::runningModule] I will not start the walking. Please "
                        "try to start again.";
            return true;
        }

        // get only the yaw axis
        iDynTree::Rotation tempRot;
        iDynTree::toEigen(tempRot) = getRotation(openXrHeadInitialTransform);

        double r, p, y;

        // This code was taken from https://www.geometrictools.com/Documentation/EulerAngles.pdf
        // Section 2.2
        auto inverseKinematicsXZY = [](const iDynTree::Rotation &chest_R_head,
                double &neckPitch,
                double &neckRoll,
                double &neckYaw)
        {
            if (chest_R_head(0,1) < +1.0)
            {
                if (chest_R_head(0, 1) > -1.0)
                {
                    neckRoll  = std::asin(-chest_R_head(0, 1)); //The roll is thetaZ
                    neckPitch = std::atan2(chest_R_head(2, 1), chest_R_head(1, 1)); //The pitch is thetaX
                    neckYaw   = std::atan2(chest_R_head(0, 2), chest_R_head(0, 0)); //The yaw is thetay
                }
                else
                {
                    neckRoll  = M_PI/2.0;
                    neckPitch = -std::atan2(-chest_R_head(2, 0), chest_R_head(2, 2));
                    neckYaw   = 0.0;
                }
            }
            else
            {
                neckRoll  = -M_PI/2.0;
                neckPitch = std::atan2(-chest_R_head(2, 0), chest_R_head(2, 2));
                neckYaw   = 0.0;
            }
        };

        inverseKinematicsXZY(tempRot, p,r,y);

        iDynTree::Transform tempTransform;
        if (m_initializationUseFullRotation)
        {
            tempTransform.setRotation(tempRot);
        } else
        {

            tempTransform.setRotation(
                iDynTree::Rotation::RotY(y)); // We remove only the initial rotation of
            // the person head around gravity.
        }
        tempTransform.setPosition(iDynTree::make_span(
                                      getPosition(openXrHeadInitialTransform))); // We remove the initial position between the
        // head and the reference frame.

        iDynTree::toEigen(m_openXrInitialAlignement)
                = iDynTree::toEigen(tempTransform.inverse().asHomogeneousTransform());
    }

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
    yInfo() << "[OculusModule::runningModule] start the robot";
    yInfo() << "[OculusModule::runningModule] Running ...";

    return true;
}

double OculusModule::evaluateDesiredFingersVelocity(unsigned int squeezeIndex,
                                                    unsigned int releaseIndex)
{
    double releaseFingersVelocity = 0.0, squeezeFingersVelocity = 0.0;
    if (m_joypadControllerInterface)
    {
        m_joypadControllerInterface->getAxis(squeezeIndex, squeezeFingersVelocity);
        m_joypadControllerInterface->getAxis(releaseIndex, releaseFingersVelocity);
    }

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

            if (m_useOpenXr)
            {
                yError() << "[OculusModule::getTransforms] If OpenXr is used the head transform "
                            "must be provided by the transform server.";
                return false;
            }

            // head
            // get head orientation
            yarp::os::Bottle* desiredHeadOrientation = NULL;
            iDynTree::Vector3 desiredHeadOrientationVector;
            desiredHeadOrientation = m_oculusOrientationPort.read(false);
            if (desiredHeadOrientation != NULL)
            {
                for (int i = 0; i < desiredHeadOrientation->size(); i++)
                    desiredHeadOrientationVector(i)
                        = iDynTree::deg2rad(desiredHeadOrientation->get(i).asFloat64());

                // Notice that the data coming from the port are written in the following order:
                // [ pitch, -roll, yaw].
                getRotation(m_oculusRoot_T_headOculus)
                    = iDynTree::toEigen(iDynTree::Rotation::RPY(-desiredHeadOrientationVector(1),
                                                                desiredHeadOrientationVector(0),
                                                                desiredHeadOrientationVector(2)));

                m_oculusHeadsetPoseInertial[3] = -desiredHeadOrientationVector(1);
                m_oculusHeadsetPoseInertial[4] = desiredHeadOrientationVector(0);
                m_oculusHeadsetPoseInertial[5] = desiredHeadOrientationVector(2);
            }

            // get head position
            yarp::os::Bottle* desiredHeadPosition = NULL;
            desiredHeadPosition = m_oculusPositionPort.read(false);
            if (desiredHeadPosition != NULL)
            {
                for (unsigned i = 0; i < desiredHeadPosition->size(); i++)
                {
                    getPosition(m_oculusRoot_T_headOculus)(i)
                        = desiredHeadPosition->get(i).asFloat64();
                }

                // the data coming from oculus vr is with the following order:
                // [x,y,z]
                // coordinate system definition is provided in:
                // https://developer.oculus.com/documentation/pcsdk/latest/concepts/dg-sensor/
                //            iDynTree::toEigen(m_oculusRoot_T_headOculus).block(0, 3, 3, 1)
                //                = iDynTree::toEigen(desiredHeadPositionVector);
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

            if (m_useOpenXr)
            {
                iDynTree::toEigen(m_oculusRoot_T_headOculus)
                    = iDynTree::toEigen(m_openXrInitialAlignement)
                      * // This is to remove any initial misplacement and rotations around gravity
                      iDynTree::toEigen(m_oculusRoot_T_headOculus);
            }

            iDynTree::Rotation temp;
            iDynTree::toEigen(temp)
                = iDynTree::toEigen(m_oculusRoot_T_headOculus).block(0, 0, 3, 3);

            temp.getRPY(m_oculusHeadsetPoseInertial[3],
                        m_oculusHeadsetPoseInertial[4],
                        m_oculusHeadsetPoseInertial[5]);
        }
    }

    // m_oculusHeadsetPoseInertial is a 6d std::vector here I'm getting the first three elements
    Eigen::Map<Eigen::Vector3d>(m_oculusHeadsetPoseInertial.data())
        = getPosition(m_oculusRoot_T_headOculus);

    if (!m_useXsens && !m_useIFeel)
    {

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

    if (!m_useXsens)
    {
        if (!m_head->controlHelper()->getFeedback())
        {
            yError() << "[OculusModule::getFeedbacks] Unable to get the joint encoders feedback: "
                        "HeadRetargeting";
            return false;
        }
        m_head->controlHelper()->updateTimeStamp();
    }

    return true;
}

bool OculusModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

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
            yWarning() << "[OculusModule::updateModule] Unable to get the transforms. Skipping iteration.";
            return true;
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
            if (m_useOpenXr)
            {
                m_head->setDesiredHeadOrientationFromOpenXr(m_oculusRoot_T_headOculus);
            } else
            {
                m_head->setDesiredHeadOrientation(m_oculusRoot_T_headOculus);
            }
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
        }
        if (!m_useXsens && !m_useIFeel)
        {
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
            double x = 0.0, y = 0.0;
            if (m_joypadControllerInterface)
            {
                m_joypadControllerInterface->getAxis(m_xJoypadIndex, x);
                m_joypadControllerInterface->getAxis(m_yJoypadIndex, y);
            }

            x = -m_scaleX * deadzone(x);
            y = m_scaleY * deadzone(y);
            std::swap(x, y);

            cmd.addString("setGoal");
            cmd.addFloat64(x);
            cmd.addFloat64(y);
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
        float buttonMapping = -1.0;

        if (m_joypadControllerInterface)
        {
            // prepare robot (A button)
            m_joypadControllerInterface->getButton(m_stopWalkingIndex, buttonMapping);
        }

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
        float buttonMapping = -1.0;

        if (m_joypadControllerInterface)
        {
            // prepare robot (A button)
            m_joypadControllerInterface->getButton(m_prepareWalkingIndex, buttonMapping);
        }
        if (buttonMapping > 0)
        {
            this->preparingModule();
        }

    } else if (m_state == OculusFSM::InPreparation)
    {
        if (!m_useXsens)
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
        }

        float buttonMapping = -1.0;
        if (m_joypadControllerInterface)
        {
            // start walking (X button)
            m_joypadControllerInterface->getButton(m_startWalkingIndex, buttonMapping);
        }

        if (buttonMapping > 0 || (m_autostart && ((yarp::os::Time::now() - m_autostartConfigureTime) > m_autostartDelay)))
        {
            this->runningModule();
        }
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

bool OculusModule::openLogger()
{
#ifdef ENABLE_LOGGER
    std::string currentTime = YarpHelper::getTimeDateMatExtension();
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
