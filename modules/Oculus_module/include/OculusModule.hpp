/**
 * @file HandRetargeting.hpp
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
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

#include <FingersRetargeting.hpp>
#include <HandRetargeting.hpp>
#include <HeadRetargeting.hpp>

/**
 * Class usefull to manage the retargeting of one hand
 */
class OculusModule : public yarp::os::RFModule
{
private:
    double m_dT; /**< Module period. */
    double m_actuatedDOFs; /**< Number of actuated DoF. */
    double m_deadzone;
    double m_fullscale;
    double m_scaleX;
    double m_scaleY;
    double m_joypadX;
    double m_joypadY;

    bool m_useHead; /**< If True the head is controlled */
    bool m_useLeftHand; /**< If True the left hand is controlled */
    bool m_useRightHand; /**< If True the right hand is controlled */
    bool m_useLeftFingers; /**< If True the left fingers are controlled */
    bool m_useRightFingers; /**< If True the right fingers are controlled */
    bool m_useOculusJoypad;
    bool m_useVirtualizer; /*using virtualizer*/
    bool m_useLeftStick;

    std::vector<std::string>
        m_axesList; /**< Vector containing the name of the controlled joints. */

    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    yarp::dev::PolyDriver m_oculusDevice; /**< Oculus device. */
    yarp::dev::IPreciselyTimed* iTimed;

    yarp::dev::IFrameTransform* m_transformClient{nullptr}; /**< Transform client. */

    // YARP Interfaces exposed by the remotecontrolboardremapper
    yarp::dev::IEncodersTimed* m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* m_positionDirectInterface{
        nullptr}; /**< Direct position control interface. */
    yarp::dev::IPositionControl2* m_positionInterface{nullptr}; /**< Position control interface. */
    // yarp::dev::IVelocityControl *m_velocityInterface{nullptr}; /**< Position control interface.
    // */
    yarp::dev::IControlMode2* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits2* m_limitsInterface{nullptr}; /**< Encorders interface. */
    yarp::os::Bottle m_remoteControlBoards; /**< Contain all the name of the controlled joints. */

    yarp::sig::Vector m_qDesired; /**< Vector containing desired joint position [deg]. */
    yarp::sig::Vector m_positionFeedbackInDegrees;
    yarp::sig::Vector m_velocityFeedbackInDegrees;
    yarp::sig::Vector m_positionFeedbackInRadians;
    yarp::sig::Vector m_minJointsPosition;
    yarp::sig::Vector m_maxJointsPosition;

    std::unique_ptr<HeadRetargeting> m_head; /**< Pointer to the head retargeting object. */
    std::unique_ptr<FingersRetargeting>
        m_leftHandFingers; /**< Pointer to the left finger retargeting object. */
    std::unique_ptr<FingersRetargeting>
        m_rightHandFingers; /**< Pointer to the right finger retargeting object. */
    std::unique_ptr<HandRetargeting>
        m_rightHand; /**< Pointer to the right hand retargeting object. */
    std::unique_ptr<HandRetargeting>
        m_leftHand; /**< Pointer to the left hand retargeting object. */

    // ports
    yarp::os::BufferedPort<yarp::sig::Vector> m_leftHandPosePort; /**< Left hand port pose. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_rightHandPosePort; /**< Right hand port pose. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_joypadOculusPort; /**< Joypad oculus port. */
    yarp::os::BufferedPort<yarp::os::Bottle>
        m_oculusOrientationPort; /**< Oculus orientation port. */
    yarp::os::BufferedPort<yarp::sig::Vector>
        m_playerOrientationPort; /**< Player orientation port. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_imagesOrientationPort;
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort;
    yarp::os::BufferedPort<yarp::sig::Vector>
        m_joypadAxisPort; /**< Port for axis data of oculus joypad for decoupling */
    yarp::os::RpcClient m_Joyrpc;

    double m_robotYaw;

    yarp::sig::Vector m_desiredHeadOrientation; /**< Vector containing the desire head orientation
                                                read by the oculus. */
    yarp::sig::Matrix m_loculus_T_rootFixed; /**< Transformation matrix between the left hand
                                             and the oculus fixed link. */
    yarp::sig::Matrix m_roculus_T_rootFixed; /**< Transformation matrix between the right hand
                                             and the oculus fixed link. */
    yarp::sig::Vector m_fingerCommands; /**< Vector containing the Joypad button values. */
    double m_playerOrientation; /**< Player orientation (read by the Virtualizer) only yaw. */

    /**
     * Get the name of the controlled joints from the resource finder
     * and set its.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool setControlledJoints(const yarp::os::Searchable& rf);

    /**
     * Configure the Robot.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configureRobot(const yarp::os::Searchable& config);

    /**
     * Configure the Oculus.
     * @return true in case of success and false otherwise.
     */
    bool configureOculus();

    /**
     * Switch the control mode.
     * @param controlMode is the control mode.
     * @return true in case of success and false otherwise.
     */
    bool switchToControlMode(const int& controlMode);

    /**
     * Set the desired position reference.
     * (The position will be sent using DirectPositionControl mode)
     * @param desiredPositionsRad desired final joint position;
     * @return true in case of success and false otherwise.
     */
    bool setDirectPositionReferences(const yarp::sig::Vector& desiredPositions);

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

public:
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;
};

#endif
