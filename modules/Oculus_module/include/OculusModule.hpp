/**
 * @file OculusModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef OCULUS_MODULE_HPP
#define OCULUS_MODULE_HPP

// std
#include <memory>

// YARP
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

#include <FingersRetargeting.hpp>
#include <HandRetargeting.hpp>
#include <HeadRetargeting.hpp>
#include <TorsoRetargeting.hpp>
//#include <WholeBodyRetargeting.hpp>

/**
 * OculusModule is the main core of the Oculus application. It is goal is to evaluate retrieve the
 * Oculus readouts, send the desired pose of the hands to the walking application, move the robot
 * fingers and move the robot head
 */
class OculusModule : public yarp::os::RFModule
{
private:
    double m_dT; /**< Module period. */

    /** Oculus Finite state machine */
    enum class OculusFSM
    {
        Configured,
        Running
    };
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
    unsigned int m_prepareWalkingIndex; /**< Index of the prepare walking button */

    bool m_useVirtualizer; /**< True if the virtualizer is used in the retargeting */
    bool m_useXsens; /**< True if the Xsens is used in the retargeting */

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

    std::unique_ptr<TorsoRetargeting> m_torso; /**< Pointer to the torso retargeting object. */

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

    yarp::os::RpcClient m_rpcWalkingClient; /**< Rpc client used for sending command to the walking
                                               controller */
    yarp::os::RpcClient
        m_rpcVirtualizerClient; /**< Rpc client used for sending command to the virtualizer */

    /** Port used to retrieve the human whole body joint pose. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_wholeBodyHumanJointsPort;

    /** Port used to retrieve the human whole body joint pose. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_wholeBodyHumanSmoothedJointsPort;

    double m_robotYaw; /**< Yaw angle of the robot base */

    yarp::sig::Matrix m_oculusRoot_T_lOculus;
    yarp::sig::Matrix m_oculusRoot_T_rOculus;
    yarp::sig::Matrix m_oculusRoot_T_headOculus;

    double m_playerOrientation; /**< Player orientation (read by the Virtualizer)
                                   only yaw. */

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

public:
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
};

#endif
