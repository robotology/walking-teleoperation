// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef OCULUS_MODULE_HPP
#define OCULUS_MODULE_HPP

// std
#include <ctime>
#include <memory>
#include <mutex>

// YARP
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>

#include <FingersRetargeting.hpp>
#include <HandRetargeting.hpp>
#include <HeadRetargeting.hpp>

#include <thrifts/TeleoperationCommands.h>

#ifdef ENABLE_LOGGER
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif

/**
 * OculusModule is the main core of the Oculus application. It is goal is to evaluate retrieve the
 * Oculus readouts, send the desired pose of the hands to the walking application, move the robot
 * fingers and move the robot head
 */
class OculusModule : public yarp::os::RFModule, public TeleoperationCommands
{
private:
    double m_dT; /**< Module period. */

    /** Oculus Finite state machine */
    enum class OculusFSM
    {
        Configured,
        Running,
        InPreparation
    };
    struct Impl;
    std::unique_ptr<Impl> pImpl;
    OculusFSM m_state; /**< State of the OculusFSM */

    // joypad utils
    // TODO move in a separate class
    double m_deadzone; /**< Joypad deadzone */
    double m_fullscale; /**< Joypad fullscale */
    double m_scaleX; /**< Scaling factor on the x axis */
    double m_scaleY; /**< Scaling factor on the y axis */
    double m_joypadX; /**< x value */
    double m_joypadY; /**< y value */

    unsigned int m_xJoypadIndex; /**< Mapping of the axis related to x coordinate */
    unsigned int m_yJoypadIndex; /**< Mapping of the axis related to y coordinate */

    unsigned int m_squeezeLeftIndex; /**< Index of the trigger used for squeezing the left hand */
    unsigned int m_squeezeRightIndex; /**< Index of the trigger used for squeezing the right hand */

    unsigned int m_releaseLeftIndex; /**< Index of the trigger used for releasing the left hand */
    unsigned int m_releaseRightIndex; /**< Index of the trigger used for releasing the right hand */

    unsigned int m_startWalkingIndex; /**< Index of the start walking button */
    unsigned int m_stopWalkingIndex; /**< Index of the stop walking button */
    unsigned int m_prepareWalkingIndex; /**< Index of the prepare walking button */

    bool m_useVirtualizer; /**< True if the virtualizer is used in the retargeting */
    bool m_useXsens; /**< True if the Xsens is used in the retargeting */
    bool m_useIFeel; /**< True if the iFeel is used in the retargeting */
    bool m_useOpenXr; /**< True if OpenXr is used in the retargeting */
    bool m_useSenseGlove; /**< True if the SenseGlove is used in the retargeting */
    bool m_skipJoypad; /**< If true, avoid connecting to the JoypadControlServer (default: false) */
    bool m_moveRobot; /**< the option to give the user the possibility to do not move the robot
                         (default :: true)*/
    bool m_autostart; /**< If true, it starts automatically without using the RPC*/

    double m_autostartDelay; /*< Delay after the configure before starting.*/
    double m_autostartConfigureTime; /*< Moment in which we started the preparation.*/
    bool m_initializationUseFullRotation; /*< The initial transform will use the entire rotation instead of just the yaw.*/

    // transform server
    yarp::dev::PolyDriver m_transformClientDevice; /**< Transform client. */
    yarp::dev::IFrameTransform* m_frameTransformInterface{nullptr}; /**< Frame transform
                                                                       interface. */
    std::string m_rootFrameName; /**< Name of the root frame used in the transform server */
    std::string m_headFrameName; /**< Name of the head frame used in the transform server (NOT
                                    SUPPORTED BY YARP)*/
    std::string
        m_leftHandFrameName; /**< Name of the left hand frame used in the transform server */
    std::string
        m_rightHandFrameName; /**< Name of the right hand frame used in the transform server */

    yarp::dev::PolyDriver m_joypadDevice; /**< Joypad polydriver. */
    yarp::dev::IJoypadController* m_joypadControllerInterface{nullptr}; /**< joypad interface. */

    std::unique_ptr<HeadRetargeting> m_head; /**< Pointer to the head retargeting object. */
    std::unique_ptr<FingersRetargeting> m_leftHandFingers; /**< Pointer to the left
                                                              finger retargeting object. */
    std::unique_ptr<FingersRetargeting> m_rightHandFingers; /**< Pointer to the right
                                                               finger retargeting object. */
    std::unique_ptr<HandRetargeting> m_rightHand; /**< Pointer to the right
                                                     hand retargeting object. */
    std::unique_ptr<HandRetargeting> m_leftHand; /**< Pointer to the left hand
                                                    retargeting object. */

    // ports
    yarp::os::BufferedPort<yarp::sig::Vector> m_leftHandPosePort; /**< Left hand port pose. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_rightHandPosePort; /**< Right hand port pose. */

    /** Player orientation port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_playerOrientationPort;
    /** Port used to simulate an imu for moving the images. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_imagesOrientationPort;
    /** Port used to retrieve the robot base orientation. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort;
    /** Port used to retrieve the headset oculus orientation. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_oculusOrientationPort;
    /** Port used to retrieve the headset oculus position. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_oculusPositionPort;

    yarp::os::RpcClient m_rpcWalkingClient; /**< Rpc client used for sending command to the walking
                                               controller */
    yarp::os::RpcClient
        m_rpcVirtualizerClient; /**< Rpc client used for sending command to the virtualizer */

    yarp::os::RpcServer m_rpcOculusServerPort; /**< RPC port to control Oculus FSM. */

    double m_robotYaw; /**< Yaw angle of the robot base */

    yarp::sig::Matrix m_oculusRoot_T_lOculus;
    yarp::sig::Matrix m_oculusRoot_T_rOculus;
    yarp::sig::Matrix m_oculusRoot_T_headOculus;
    yarp::sig::Matrix m_openXrInitialAlignement;

    std::vector<double> m_oculusHeadsetPoseInertial;

    double m_playerOrientation{0}; /**< Player orientation (read by the Virtualizer)
                                   only yaw. */
    double m_playerOrientationOld{0}; /**< previous updated Player orientation (read by the
                                   Virtualizer) only yaw. */
    double m_playerOrientationThreshold; /**< Player orientation threshold. */

    bool m_enableLogger; /**< log the data (if ON) */

    std::mutex m_mutex; /**< Mutex. */

#ifdef ENABLE_LOGGER
    XBot::MatLogger2::Ptr m_logger; /**< */
    XBot::MatAppender::Ptr m_appender;
    std::string m_logger_prefix{"oculus"};
#endif
    /**
     * Configure the Oculus.
     * @param config configuration object
     * @return true in case of success and false otherwise.
     */
    bool configureOculus(const yarp::os::Searchable& config);

    /**
     * Configure the Tranformation Client.
     * @param config configuration object
     * @return true in case of success and false otherwise.
     */
    bool configureTranformClient(const yarp::os::Searchable& config);

    /**
     * Configure the Joypad.
     * @param config configuration object
     * @return true in case of success and false otherwise.
     */
    bool configureJoypad(const yarp::os::Searchable& config);

    /**
     * @brief Reset a robot camera to its default settings.
     * @param cameraPort The remote port to the camera
     * @param localPort The local port needed for the driver to open
     * @return true in case of success and false otherwise.
     */
    bool resetCamera(const std::string& cameraPort, const std::string& localPort);

    /**
     * @brief Set a robot camera settings to the automatic mode.
     * @param cameraPort The remote port to the camera
     * @param localPort The local port needed for the driver to open
     * @return true in case of success and false otherwise.
     */
    bool setCameraAutoMode(const std::string& cameraPort, const std::string& localPort);

    /**
     * Get all the feedback signal from the interfaces
     * @return true in case of success and false otherwise.
     */
    bool getFeedbacks();

    /**
     * Implements a deadzone for the joypad
     * @return joypad inputs after applying deadzone.
     */
    double deadzone(const double&);

    /**
     * Evaluate the desired fingers velocity
     * @param squeezeIndex index used for squeezing
     * @param releaseIndex index used for releasing
     * @return the the desired joints velocity for the fingers in radiant / seconds
     */
    double evaluateDesiredFingersVelocity(unsigned int squeezeIndex, unsigned int releaseIndex);

    /**
     * Get the transformation from the transform server
     * @return true in case of success and false otherwise.
     */
    bool getTransforms();

    /**
     * Open the logger
     * @return true if it could open the logger
     */
    bool openLogger();

public:
    OculusModule();
    ~OculusModule();
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() final;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() final;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) final;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() final;

    /**
     * This method prepares the Oculus Module for Teleoperation.
     * @return true/false in case of success/failure;
     */
    virtual bool prepareTeleoperation() override;

    /**
     * This method runs the entire Oculus Module for Teleoperation.
     * @return true/false in case of success/failure;
     */
    virtual bool runTeleoperation() override;

    /**
     * This method is called in order to prepare the Oculus Module FSM for Teleoperation.
     * @return true/false in case of success/failure;
     */
    bool preparingModule();

    /**
     * This method is called in order to run the Oculus Module FSM for Teleoperation.
     * @return true/false in case of success/failure;
     */
    bool runningModule();

    /**
     * to get the Oculus FSM in string format
     * @param OculusFSM to get the string value
     * @return string associated with the inpute sate
     */
    std::string getStringFromOculusState(const OculusFSM state);
};

#endif
