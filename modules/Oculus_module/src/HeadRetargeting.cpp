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

// This code was taken from https://www.geometrictools.com/Documentation/EulerAngles.pdf
// Section 2.3
void HeadRetargeting::inverseKinematics(const iDynTree::Rotation& chest_R_head,
                                        double& neckPitch,
                                        double& neckRoll,
                                        double& neckYaw)
{
    // YXZ decomposition
    if (chest_R_head(1, 2) < 1)
    {
        if (chest_R_head(1, 2) > -1)
        {
            neckRoll = std::asin(-chest_R_head(1, 2));
            neckPitch = std::atan2(chest_R_head(0, 2), chest_R_head(2, 2));
            neckYaw = std::atan2(chest_R_head(1, 0), chest_R_head(1, 1));
        } else
        {
            neckRoll = iDynTree::deg2rad(90);
            neckPitch = -std::atan2(-chest_R_head(0, 1), chest_R_head(0, 0));
            neckYaw = 0;
        }
    } else
    {
        neckRoll = -iDynTree::deg2rad(90);
        neckPitch = std::atan2(-chest_R_head(0, 1), chest_R_head(0, 0));
        neckYaw = 0;
    }

    // minus due to the joints mechanism of the iCub neck
    neckRoll = -neckRoll;
    return;
}

iDynTree::Rotation HeadRetargeting::forwardKinematics(const double& neckPitch,
                                                      const double& neckRoll,
                                                      const double& neckYaw)
{
    iDynTree::Rotation chest_R_head;
    chest_R_head = iDynTree::Rotation::RotY(neckPitch) * iDynTree::Rotation::RotX(-neckRoll)
                   * iDynTree::Rotation::RotZ(neckYaw);

    return chest_R_head;
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

    return true;
}

void HeadRetargeting::setPlayerOrientation(const double& playerOrientation)
{
    // notice in this case the real transformation is rotx(-pi) * rotz(playerOrientation) * rotx(pi)
    // which is equal to rotz(-playerOrietation);
    m_oculusInertial_R_teleopFrame = iDynTree::Rotation::RotZ(-playerOrientation);
}

void HeadRetargeting::setDesiredHeadOrientation(
    const yarp::sig::Matrix& oculusInertial_T_headOculus)
{
    // get the rotation matrix
    iDynTree::toEigen(m_oculusInertial_R_headOculus)
        = iDynTree::toEigen(oculusInertial_T_headOculus).block(0, 0, 3, 3);
}

bool HeadRetargeting::move()
{
    return RetargetingController::move();
}

void HeadRetargeting::evalueNeckJointValues()
{

    m_teleopFrame_R_headOculus
        = m_oculusInertial_R_teleopFrame.inverse() * m_oculusInertial_R_headOculus;

    // notice here the following assumption is done:
    // desiredNeckJoint(0) = neckPitch
    // desiredNeckJoint(1) = neckRoll
    // desiredNeckJoint(2) = neckYaw
    yarp::sig::Vector desiredNeckJoint(3);
    inverseKinematics(
        m_teleopFrame_R_headOculus, desiredNeckJoint(0), desiredNeckJoint(1), desiredNeckJoint(2));

    // Notice: this can generate problems when the inverse kinematics return angles
    // near the singularity. it would be nice to implement a smoother in SO(3).
    m_headTrajectorySmoother->computeNextValues(desiredNeckJoint);
    m_desiredJointValue = m_headTrajectorySmoother->getPos();
}

bool HeadRetargeting::initializeNeckJointValues()
{
    m_desiredJointValue.clear();
    m_desiredJointValue.resize(
        static_cast<size_t>(RetargetingController::controlHelper()->getDoFs()), 0.0);
    return RetargetingController::controlHelper()->initializeJointValues(m_desiredJointValue);
}

void getNeckJointValues();

void HeadRetargeting::getNeckJointValues(std::vector<double>& neckValues)
{
    neckValues.clear();
    for (size_t i = 0; i < m_desiredJointValue.size(); i++)
        neckValues.push_back(m_desiredJointValue[i]);
}
