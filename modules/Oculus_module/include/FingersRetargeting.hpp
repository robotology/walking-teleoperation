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

using namespace yarp::math;

/**
 * Class useful to manage the retargeting of one hand
 */
class FingersRetargeting
{
private:
    double m_fingersJoints; /**< Contain the number of fingers. */
    yarp::sig::Vector m_fingerVelocity; /**< It contains the desired finger velocity. */
    std::unique_ptr<iCub::ctrl::Integrator> m_fingerIntegrator{nullptr};

    yarp::sig::Vector m_desiredPosition;

public:
    /**
     * Configure the object.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config);

    /**
     * Close the hand
     * @return true in case of success and false otherwise.
     */
    bool closeHand();

    /**
     * Open the hand
     * @param desiredPosition desired fingers position [out].
     * @return true in case of success and false otherwise.
     */
    bool openHand();

    /**
     * Return fingers position.
     * @return fingers position
     */
    const yarp::sig::Vector& fingerPosition() const;
};
#endif
