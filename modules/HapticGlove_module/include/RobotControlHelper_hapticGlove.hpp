/**
 * @file HapticGloveRobotControlHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef ROBOT_CONTROL_HELPER_HAPTIC_GLOVE_HPP
#define ROBOT_CONTROL_HELPER_HAPTIC_GLOVE_HPP

// std
#include <memory>

// YARP
#include <yarp/sig/Vector.h>

#include <RobotControlInterface_hapticGlove.hpp>

using namespace yarp::math;
// using namespace HapticGlove;

/**
 * RobotControlHelper is a virtual class for helping to control one part of the robot (i.e. head or
 * fingers)
 */
class RobotControlHelper
{
protected:
    std::unique_ptr<HapticGlove::RobotControlInterface>
        m_robotControlInterface; /**< Controller helper */
    yarp::sig::Vector m_desiredMotorValue; /** Desired Motor value in radiant or radiant/s  */
    yarp::sig::Vector m_desiredJointValue; /** Desired joint value in radiant or radiant/s  */

public:
    /**
     * Configure the object.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    virtual bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand)
        = 0;

    /**
     * Move the robot part
     * @return true in case of success and false otherwise.
     */
    virtual bool move();

    /**
     * Expose the contolHelper interface (const)
     * @return control helper interface
     */
    const std::unique_ptr<HapticGlove::RobotControlInterface>& controlHelper() const;

    /**
     * Expose the contolHelper interface
     * @return control helper interface
     */
    std::unique_ptr<HapticGlove::RobotControlInterface>& controlHelper();
    virtual ~RobotControlHelper();
};
#endif
