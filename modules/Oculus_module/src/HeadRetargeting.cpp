/**
 * @file HeadRetargeting.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <HeadRetargeting.hpp>
#include <Utils.hpp>

bool HeadRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    // check if the configuration file is empty
    if (config.isNull())
    {
        yError() << "[configure] Empty configuration for head retargeting.";
        return false;
    }

    m_controlHelper == std::make_unique<RobotControlHelper>();
    if (!m_controlHelper->configure(config, name))
    {
        yError() << "[FingersRetargeting::configure] Unable to configure the finger helper";
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
    // notice in this case the real transformation is rotx(-pi) * rotz(playerOrientation) * rotx(pi)
    // which is equal to rotz(-playerOrietation);
    m_playerOrientation = iDynTree::Rotation::RotZ(-playerOrientation);
}

void HeadRetargeting::setDesiredHeadOrientation(const yarp::sig::Matrix& desiredHeadTransform)
{
    iDynTree::toEigen(m_desiredHeadOrientation)
        = iDynTree::toEigen(desiredHeadTransform).block(0, 0, 3, 3);
}

void HeadRetargeting::evaluateHeadOrientationCorrected()
{
    iDynTree::Rotation desiredHeadOrientation;
    desiredHeadOrientation = m_playerOrientation.inverse() * m_desiredHeadOrientation;

    yarp::sig::Vector tmp(3);
    iDynTree::toYarp(desiredHeadOrientation.asRPY(), tmp);

    m_headTrajectorySmoother->computeNextValues(tmp);
    m_desiredJointPosition = m_headTrajectorySmoother->getPos();
}
