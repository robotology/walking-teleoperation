/**
 * @file HapticGloveModule.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef HAPTIC_GLOVE_MODULE_HPP
#define HAPTIC_GLOVE_MODULE_HPP

#define _USE_MATH_DEFINES
#include <cmath>

// std
#include <chrono>
#include <ctime>
#include <memory>
#include <vector>

// YARP
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

#include <GloveControlHelper.hpp>
#include <RobotController_hapticGlove.hpp>
#include <Teleoperation.hpp>

#include <Retargeting.hpp>
#include <iCub/ctrl/minJerkCtrl.h>

/**
 * OculusModule is the main core of the Oculus application. It is goal is to evaluate retrieve the
 * Oculus readouts, send the desired pose of the hands to the walking application, move the robot
 * fingers and move the robot head
 */

class HapticGloveModule : public yarp::os::RFModule
{
private:
    std::string m_logPrefix;
    double m_dT; /**< Module period. */
    std::string m_robot; /**< robot name. */

    /** Haptic Glove Finite state machine */
    enum class HapticGloveFSM
    {
        Configuring,
        Running,
        Preparing
    };

    HapticGloveFSM m_state; /**< State of the HapticGloveFSM */

    bool m_useLeftHand,
        m_useRightHand; /**< use the specided hand if the flag is ON (default value is ON)*/

    std::unique_ptr<HapticGlove::Teleoperation> m_leftHand;
    std::unique_ptr<HapticGlove::Teleoperation> m_rightHand;

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
