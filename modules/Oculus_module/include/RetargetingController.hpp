// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef RETARGETING_CONTROLLER_HPP
#define RETARGETING_CONTROLLER_HPP

// std
#include <memory>

// YARP
#include <yarp/sig/Vector.h>

#include <RobotControlHelper.hpp>

using namespace yarp::math;

/**
 * RetargetingController is a virtual class for retargeting one part of the robot (i.e. head or
 * fingers)
 */
class RetargetingController
{
protected:
    std::unique_ptr<RobotControlHelper> m_controlHelper; /**< Controller helper */
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
    const std::unique_ptr<RobotControlHelper>& controlHelper() const;

    /**
     * Expose the contolHelper interface
     * @return control helper interface
     */
    std::unique_ptr<RobotControlHelper>& controlHelper();
    virtual ~RetargetingController();
};
#endif
