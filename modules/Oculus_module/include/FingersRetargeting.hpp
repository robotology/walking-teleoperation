// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FINGERS_RETARGETING_HPP
#define FINGERS_RETARGETING_HPP

// std
#include <memory>

// YARP
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

#include <RetargetingController.hpp>

using namespace yarp::math;

/**
 * Class useful to manage the retargeting of fingers.
 */
class FingersRetargeting : public RetargetingController
{
private:
    yarp::sig::Vector m_fingersScaling; /**< It contains the finger velocity scaling. */

    std::unique_ptr<iCub::ctrl::Integrator> m_fingerIntegrator{nullptr}; /**< Velocity integrator */

public:
    /**
     * Configure the object.
     * @param config reference to a resource finder object.
     * @param name name of the robot
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    /**
     * Set the fingers velocities
     * @param fingersVelocity value from -1 to 1
     * @return true in case of success and false otherwise.
     */
    bool setFingersVelocity(const double& fingersVelocity);

    /**
     * Get the fingers velocities or values
     * @param fingerValue get the finger velocity or value
     */
    void getFingerValues(std::vector<double>& fingerValues);
};
#endif
