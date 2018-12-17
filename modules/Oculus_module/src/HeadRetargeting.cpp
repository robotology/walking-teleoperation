/**
 * @file HeadRetargeting.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include "HeadRetargeting.hpp"
#include "Utils.hpp"

bool HeadRetargeting::configure(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if (config.isNull())
    {
        yError() << "[configure] Empty configuration for head retargeting.";
        return false;
    }

    // initialize minimum jerk trajectory for the head
    double samplingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[configure] Unable to find the head sampling time";
        return false;
    }

    double smoothingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "smoothingTime", smoothingTime))
    {
        yError() << "[configure] Unable to find the head smoothing time";
        return false;
    }

    m_headTrajectorySmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(3, samplingTime, smoothingTime);
    yarp::sig::Vector buff(3, 0.0);
    m_headTrajectorySmoother->init(buff);

    return true;
}

void HeadRetargeting::setPlayerOrientation(const double& playerOrientation)
{
    m_playerOrientation = playerOrientation;
}

void HeadRetargeting::setDesiredHeadOrientation(const yarp::sig::Vector& desiredHeadOrientation)
{
    m_desiredHeadOrientation = desiredHeadOrientation;
}

void HeadRetargeting::evaluateHeadOrientationCorrected()
{
    yarp::sig::Vector desiredHeadOrientation;
    desiredHeadOrientation = m_desiredHeadOrientation;
    desiredHeadOrientation(2) = desiredHeadOrientation(2) + m_playerOrientation;

    m_headTrajectorySmoother->computeNextValues(desiredHeadOrientation);
}

yarp::sig::Vector HeadRetargeting::getHeadOrientation()
{
    return m_headTrajectorySmoother->getPos();
}
