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

iDynTree::Vector3 HeadRetargeting::inverseKinematics(const iDynTree::Rotation& matrix)
{
    // YXZ decomposition
    iDynTree::Vector3 YXZ;
    double thetaX, thetaY, thetaZ;
    if (matrix(1, 2) < 1)
    {
        if (matrix(1, 2) > -1)
        {
            thetaX = std::asin(-matrix(1, 2));
            thetaY = std::atan2(matrix(0, 2), matrix(2, 2));
            thetaZ = std::atan2(matrix(1, 0), matrix(1, 1));
        } else
        {
            thetaX = M_PI / 2;
            thetaY = -std::atan2(-matrix(0, 1), matrix(0, 0));
            thetaZ = 0;
        }
    } else
    {
        thetaX = -M_PI / 2;
        thetaY = std::atan2(-matrix(0, 1), matrix(0, 0));
        thetaZ = 0;
    }

    // minus due to the joints structure of the iCub neck
    YXZ(0) = -thetaY;
    YXZ(1) = thetaX;
    YXZ(2) = thetaZ;

    return YXZ;
}

iDynTree::Rotation HeadRetargeting::forwardKinematics(const yarp::sig::Vector& YXZ)
{
    iDynTree::Rotation root_R_head;
    root_R_head = iDynTree::Rotation::RotY(-YXZ(0)) * iDynTree::Rotation::RotX(YXZ(1))
                  * iDynTree::Rotation::RotZ(YXZ(2));

    return root_R_head;
}

bool HeadRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    // check if the configuration file is empty
    if (config.isNull())
    {
        yError() << "[HeadRetargeting::configure] Empty configuration for head retargeting.";
        return false;
    }

    m_controlHelper = std::make_unique<RobotControlHelper>();
    if (!m_controlHelper->configure(config, name, true))
    {
        yError() << "[FingersRetargeting::configure] Unable to configure the finger helper";
        return false;
    }

    // initialize minimum jerk trajectory for the head
    double samplingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[HeadRetargeting::configure] Unable to find the head sampling time";
        return false;
    }

    double smoothingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "smoothingTime", smoothingTime))
    {
        yError() << "[HeadRetargeting::configure] Unable to find the head smoothing time";
        return false;
    }

    m_headTrajectorySmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(3, samplingTime, smoothingTime);
    yarp::sig::Vector buff(3, 0.0);
    m_headTrajectorySmoother->init(buff);

    if (!m_controlHelper->switchToControlMode(VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "[HeadRetargeting::configure] Unable to switch the control mode";
        return false;
    }

    return true;
}

void HeadRetargeting::setPlayerOrientation(const double& playerOrientation)
{
    // notice in this case the real transformation is rotx(-pi) * rotz(playerOrientation) * rotx(pi)
    // which is equal to rotz(-playerOrietation);
    m_playerOrientation = iDynTree::Rotation::RotZ(-playerOrientation);
}

void HeadRetargeting::setDesiredHeadOrientation(const yarp::sig::Matrix& oculusRoot_T_oculusHeadset)
{
    iDynTree::toEigen(m_oculusRoot_T_oculusHeadset)
        = iDynTree::toEigen(oculusRoot_T_oculusHeadset).block(0, 0, 3, 3);
}

bool HeadRetargeting::move()
{
    m_desiredHeadOrientation = m_playerOrientation.inverse() * m_oculusRoot_T_oculusHeadset;

    yarp::sig::Vector tmp(3);
    iDynTree::toYarp(inverseKinematics(m_desiredHeadOrientation), tmp);

    m_headTrajectorySmoother->computeNextValues(tmp);
    m_desiredJointPosition = m_headTrajectorySmoother->getPos();

    return RetargetingHelper::move();
}
