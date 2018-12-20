/**
 * @file FingersRetargeting.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
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

#include <RetargetingHelper.hpp>

using namespace yarp::math;

/**
 * Class useful to manage the retargeting of fingers.
 */
class FingersRetargeting : public RetargetingHelper
{
private:
    yarp::sig::Vector m_fingersScaling; /**< It contains the finger velocity scaling. */

    std::unique_ptr<iCub::ctrl::Integrator> m_fingerIntegrator{nullptr};

public:
    /**
     * Configure the object.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    /**
     * Move the fingers
     * @param fingersVelocity value  from -1 to 1
     * @return true in case of success and false otherwise.
     */
    bool setFingersVelocity(const double& fingersVelocity);
};
#endif
