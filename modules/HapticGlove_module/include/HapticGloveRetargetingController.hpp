/**
 * @file HapticGloveRetargetingController.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef RETARGETING_CONTROLLER_HPP
#define RETARGETING_CONTROLLER_HPP

// std
#include <memory>

// YARP
#include <yarp/sig/Vector.h>

#include <HapticGloveRobotControlHelper.hpp>

using namespace yarp::math;
// using namespace HapticGlove;

/**
 * RetargetingController is a virtual class for retargeting one part of the robot (i.e. head or
 * fingers)
 */
class RetargetingController
{
protected:
    std::unique_ptr<HapticGlove::RobotControlHelper> m_controlHelper; /**< Controller helper */
    yarp::sig::Vector m_desiredJointValue; /** Desired joint value in radiant or radiant/s  */

public:
    /**
     * Configure the object.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    virtual bool configure(const yarp::os::Searchable& config, const std::string& name) = 0;

    /**
     * Move the robot part
     * @return true in case of success and false otherwise.
     */
    virtual bool move();

    /**
     * Expose the contolHelper interface (const)
     * @return control helper interface
     */
    const std::unique_ptr<HapticGlove::RobotControlHelper>& controlHelper() const;

    /**
     * Expose the contolHelper interface
     * @return control helper interface
     */
    std::unique_ptr<HapticGlove::RobotControlHelper>& controlHelper();
    virtual ~RetargetingController();
};
#endif
