/**
 * @file HapticGloveFingersRetargeting.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef FINGERS_RETARGETING_HPP
#define FINGERS_RETARGETING_HPP

// std
#include <memory>

// YARP
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

#include <HapticGloveRetargetingController.hpp>

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
     * Set the fingers reference value
     * @param fingersReference the reference value for the finger to follow
     * @return true in case of success and false otherwise.
     */
    bool setFingersAxisReference(const double& fingersReference);

    /**
     * Get the fingers' axis velocities or values
     * @param fingerValue get the fingers' axis velocity or value
     */
    void getFingerAxisMeasuredValues(yarp::sig::Vector& fingerValues);

    /**
     * Get the fingers joint velocities or values
     * @param fingerValue get the finger joint velocity or value
     */
    void getFingerJointsMeasuredValues(yarp::sig::Vector& fingerValues);

    /**
     * Update the feedback values
     */
    bool updateFeedback(void);
};
#endif
