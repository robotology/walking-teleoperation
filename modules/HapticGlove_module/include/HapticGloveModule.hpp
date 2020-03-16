/**
 * @file HapticGloveModule.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef HAPTIC_GLOVE_MODULE_HPP
#define HAPTIC_GLOVE_MODULE_HPP

// std
#include <ctime>
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

#include <HapticGloveFingersRetargeting.hpp>

/**
 * OculusModule is the main core of the Oculus application. It is goal is to evaluate retrieve the
 * Oculus readouts, send the desired pose of the hands to the walking application, move the robot
 * fingers and move the robot head
 */

class HapticGloveModule : public yarp::os::RFModule
{
private:
    double m_dT; /**< Module period. */

    /** Haptic Glove Finite state machine */
    enum class HapticGloveFSM
    {
        Configured,
        Running,
        InPreparation
    };

    HapticGloveFSM m_state; /**< State of the HapticGloveFSM */

    bool m_moveRobot; /**< the option to give the user the possibility to do not move the robot
                         (default :: true)*/

    std::unique_ptr<FingersRetargeting> m_leftHandFingers; /**< Pointer to the left
                                                              finger retargeting object. */
    std::unique_ptr<FingersRetargeting> m_rightHandFingers; /**< Pointer to the right
                                                               finger retargeting object. */

    /**
     * Get all the feedback signal from the interfaces
     * @return true in case of success and false otherwise.
     */
    bool getFeedbacks();

    /**
     * Evaluate the desired robot fingers reference (velocity or position)
     * @return if the evaluation is done successfully
     */
    bool evaluateRobotFingersReferences();

public:
    HapticGloveModule();
    ~HapticGloveModule();
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
