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

/**
 * Class usefull to manage the retargeting of one hand
 */
class OculusModule : public yarp::os::RFModule
{
private:
    double m_dT; /**< Module period. */

    // joypad utils
    double m_deadzone;
    double m_fullscale;
    double m_scaleX;
    double m_scaleY;
    double m_joypadX;
    double m_joypadY;

    unsigned int m_xJoypadIndex;
    unsigned int m_yJoypadIndex;

    unsigned int m_squeezeLeftIndex;
    unsigned int m_squeezeRightIndex;

    unsigned int m_releaseLeftIndex;
    unsigned int m_releaseRightIndex;

    bool m_useLeftHand; /**< If True the left hand is controlled */
    bool m_useRightHand; /**< If True the right hand is controlled */
    bool m_useVirtualizer; /*using virtualizer*/

    // transform server
    yarp::dev::PolyDriver m_transformClientDevice; /**< Transform client. */
    yarp::dev::IFrameTransform* m_frameTransformInterface{nullptr}; /**< Frame transform
                                                                       interface. */

    std::string m_rootFrameName;
    std::string m_headFrameName;
    std::string m_leftHandFrameName;
    std::string m_rightHandFrameName;

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

    yarp::os::BufferedPort<yarp::sig::Vector> m_playerOrientationPort; /**< Player
                                                                          orientation port. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_imagesOrientationPort;
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort;
    yarp::os::RpcClient m_Joyrpc;

    double m_robotYaw;

    yarp::sig::Matrix m_oculusRoot_T_lOculus;
    yarp::sig::Matrix m_oculusRoot_T_rOculus;
    yarp::sig::Matrix m_oculusRoot_T_headOculus;

    double m_playerOrientation; /**< Player orientation (read by the Virtualizer)
                                   only yaw. */

    /**
     * Configure the Oculus.
     * @return true in case of success and false otherwise.
     */
    bool configureOculus(const yarp::os::Searchable& config);

    bool configureTranformClient(const yarp::os::Searchable& config);

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

    double evaluateDesiredFingersVelocity(unsigned int squeezeIndex, unsigned int releaseIndex);

    bool getTransforms();

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
