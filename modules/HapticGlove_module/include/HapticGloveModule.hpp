// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HAPTIC_GLOVE_MODULE_HPP
#define HAPTIC_GLOVE_MODULE_HPP

// std
#include <memory>

// YARP
#include <yarp/os/RFModule.h>

// teleoperation
#include <Teleoperation.hpp>

/**
 * HapticGloveModule is the main core application of the bilateral teleoperation of the human hand
 * and robot hand . It is goal is to evaluate retrieve the sense glove readouts, send the desired
 * axis values to the robot hand, and provide the force feedback and vibrotactile feedback to the
 * human.
 */
class HapticGloveModule : public yarp::os::RFModule
{
private:
    std::string m_logPrefix;
    double m_dT; /**< Module period. */
    std::string m_robot; /**< robot name. */
    double m_waitingStartTime; /**< the moment which waiting started */
    double m_waitingDurationTime; /**< the duration of waiting state machine */

    /** Haptic Glove Finite state machine */
    enum class HapticGloveFSM
    {
        Configuring,
        Running,
        Preparing,
        Waiting
    };

    HapticGloveFSM m_state; /**< State of the HapticGloveFSM */

    bool m_useLeftHand;
    bool m_useRightHand; /**< use the specided hand if the flag is ON (default value is ON)*/

    std::unique_ptr<HapticGlove::Teleoperation> m_leftHand;
    std::unique_ptr<HapticGlove::Teleoperation> m_rightHand;

public:
    /**
     * Constructor
     */
    HapticGloveModule();

    /**
     * Destructor
     * */
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

#endif // HAPTIC_GLOVE_MODULE_HPP
