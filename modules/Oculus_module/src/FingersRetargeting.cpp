/**
 * @file FingersRetargeting.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include "FingersRetargeting.hpp"
#include "Utils.hpp"


bool FingersRetargeting::configure(const yarp::os::Searchable &config)
{
    // TODO do it in a better way
    m_fingersJoints = 7;
    double samplingTime;
    if(!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[configure] Unable to find the head smoothing time";
        return false;
    }

    m_fingerVelocity.resize(m_fingersJoints);
    if(!YarpHelper::getYarpVectorFromSearchable(config, "fingersVelocity", m_fingerVelocity))
    {
        yError() << "[configure] Initialization failed while reading fingersVelocity vector.";
        return false;
    }

    m_desiredPosition.resize(m_fingersJoints);
    yarp::sig::Vector buff(m_fingersJoints, 0.0);
    m_fingerIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buff);
    return true;
}

bool FingersRetargeting::closeHand()
{
    if(m_fingerIntegrator == nullptr)
    {
        yError() << "[closeHand] the integrator is not initialized please call configure() method";
        return false;
    }

    // TODO check the limits.
    m_desiredPosition = m_fingerIntegrator->integrate(m_fingerVelocity);
    return true;
}

bool FingersRetargeting::openHand()
{
    if(m_fingerIntegrator == nullptr)
    {
        yError() << "[closeHand] the integrator is not initialized please call configure() method";
        return false;
    }

    // TODO check the limits.
    m_desiredPosition = m_fingerIntegrator->integrate(-1*m_fingerVelocity);
    return true;
}

const yarp::sig::Vector& FingersRetargeting::fingerPosition() const
{
    return m_desiredPosition;
}
