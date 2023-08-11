// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <HeadRetargeting.hpp>
#include <Utils.hpp>

struct HeadRetargeting::Impl
{
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_NeckJointsPreparationSmoother{nullptr};
    yarp::sig::Vector m_preparationJointReferenceValues;
    void initializeNeckJointsSmoother(const unsigned actuatedDOFs,
                                      const double dT,
                                      const double smoothingTime,
                                      const yarp::sig::Vector jointsInitialValue);
    void getNeckJointsRefSmoothedValues(yarp::sig::Vector& smoothedJointValues);
};

HeadRetargeting::HeadRetargeting()
    : pImpl{new Impl()} {};

HeadRetargeting::~HeadRetargeting(){};

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

// This code was taken from https://www.geometrictools.com/Documentation/EulerAngles.pdf
// Section 2.2
void HeadRetargeting::inverseKinematicsXZY(const iDynTree::Rotation &chest_R_head,
                                           double &neckPitch,
                                           double &neckRoll,
                                           double &neckYaw)
{
    if (chest_R_head(0,1) < +1.0)
    {
        if (chest_R_head(0, 1) > -1.0)
        {
            neckRoll  = std::asin(-chest_R_head(0, 1)); //The roll is thetaZ
            neckPitch = std::atan2(chest_R_head(2, 1), chest_R_head(1, 1)); //The pitch is thetaX
            neckYaw   = std::atan2(chest_R_head(0, 2), chest_R_head(0, 0)); //The yaw is thetay
        }
        else
        {
            neckRoll  = M_PI/2.0;
            neckPitch = -std::atan2(-chest_R_head(2, 0), chest_R_head(2, 2));
            neckYaw   = 0.0;
        }
    }
    else
    {
        neckRoll  = -M_PI/2.0;
        neckPitch = std::atan2(-chest_R_head(2, 0), chest_R_head(2, 2));
        neckYaw   = 0.0;
    }

    // minus due to the joints mechanism of the iCub neck
    neckRoll = -neckRoll;
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

    double preparationSmoothingTime;
    if (!YarpHelper::getDoubleFromSearchable(
            config, "PreparationSmoothingTime", preparationSmoothingTime))
    {
        yError() << "[HeadRetargeting::configure] Unable to find the head smoothing time";
        return false;
    }
    unsigned headDoFs = controlHelper()->getDoFs();

    yarp::sig::Vector preparationJointReferenceValues;
    preparationJointReferenceValues.resize(headDoFs);

    if (!YarpHelper::getYarpVectorFromSearchable(
            config, "PreparationJointReferenceValues", preparationJointReferenceValues))
    {
        yError() << "[HeadRetargeting::configure] Initialization failed while reading "
                    "PreparationJointReferenceValues vector.";
        return false;
    }

    m_headTrajectorySmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(headDoFs, samplingTime, smoothingTime);
    yarp::sig::Vector buff(headDoFs, 0.0);
    m_headTrajectorySmoother->init(buff);

    yarp::sig::Vector neckJointsFbk;
    getNeckJointValues(neckJointsFbk);
    pImpl->initializeNeckJointsSmoother(
        headDoFs, samplingTime, preparationSmoothingTime, neckJointsFbk);
    pImpl->m_preparationJointReferenceValues = preparationJointReferenceValues;

    m_playerOrientation = 0;

    m_desiredNeckJointsBeforeSmoothing.resize(3);

    return true;
}

void HeadRetargeting::setPlayerOrientation(const double& playerOrientation)
{
    // The orientation coming from the virtualizer is around an axis pointing down
    m_playerOrientation = -playerOrientation;
}

void HeadRetargeting::setDesiredHeadOrientation(
    const yarp::sig::Matrix& oculusInertial_T_headOculus)
{
    // get the rotation matrix
    iDynTree::toEigen(m_oculusInertial_R_headOculus)
        = iDynTree::toEigen(oculusInertial_T_headOculus).block(0, 0, 3, 3);

    m_teleopFrame_R_headOculus
        = iDynTree::Rotation::RotZ(m_playerOrientation).inverse() * m_oculusInertial_R_headOculus;

    // notice here the following assumption is done:
    // desiredNeckJoint(0) = neckPitch
    // desiredNeckJoint(1) = neckRoll
    // desiredNeckJoint(2) = neckYaw
    inverseKinematics(
        m_teleopFrame_R_headOculus, m_desiredNeckJointsBeforeSmoothing(0),
                                    m_desiredNeckJointsBeforeSmoothing(1),
                                    m_desiredNeckJointsBeforeSmoothing(2));

    smoothNeckJointValues();
}

void HeadRetargeting::setDesiredHeadOrientationFromOpenXr(const yarp::sig::Matrix &openXrInertial_T_headOpenXr)
{
    // get the rotation matrix
    iDynTree::toEigen(m_oculusInertial_R_headOculus)
        = iDynTree::toEigen(openXrInertial_T_headOpenXr).block(0, 0, 3, 3);

    m_teleopFrame_R_headOculus
        = iDynTree::Rotation::RotY(m_playerOrientation).inverse() //With OpenXr the Y is up
            * m_oculusInertial_R_headOculus;

    // notice here the following assumption is done:
    // desiredNeckJoint(0) = neckPitch, the angle around X, with X pointing right
    // desiredNeckJoint(1) = neckRoll, the angle around Z, with Z pointing backward
    // desiredNeckJoint(2) = neckYaw, the angle around Y, with Y pointing up
    // The kinematic chain from the chest to the neck is composed of the pitch, roll, and yaw angles, in this order. 
    // The neck pitch axis is aligned with the x axis of the reference frame used by openxr, the roll with the z axis,
    // and the yaw with the y axis. Hence, we need to find the Euler angles corresponding to R_x * R_z * R_y.
    inverseKinematicsXZY(
        m_teleopFrame_R_headOculus, m_desiredNeckJointsBeforeSmoothing(0),
                                    m_desiredNeckJointsBeforeSmoothing(1),
                                    m_desiredNeckJointsBeforeSmoothing(2));

    smoothNeckJointValues();
}

bool HeadRetargeting::move()
{
    return RetargetingController::move();
}

void HeadRetargeting::smoothNeckJointValues()
{

    // Notice: this can generate problems when the inverse kinematics return angles
    // near the singularity. it would be nice to implement a smoother in SO(3).
    m_headTrajectorySmoother->computeNextValues(m_desiredNeckJointsBeforeSmoothing);
    m_desiredJointValue = m_headTrajectorySmoother->getPos();
}

void HeadRetargeting::initializeNeckJointValues()
{
    m_desiredJointValue.clear();
    pImpl->getNeckJointsRefSmoothedValues(m_desiredJointValue);
}

void HeadRetargeting::setDesiredNeckjointsValues(yarp::sig::Vector& desiredNeckValues)
{
    m_desiredJointValue.clear();
    m_desiredJointValue = desiredNeckValues;
    yInfo() << "head desired joint values: " << m_desiredJointValue(0) << " "
            << m_desiredJointValue(1) << " " << m_desiredJointValue(2);
}

void HeadRetargeting::getNeckJointValues(yarp::sig::Vector& neckValues)
{
    neckValues.clear();
    if (!controlHelper()->getFeedback())
    {
        yInfo() << "[HeadRetargeting::getNeckJointValues] Unable the get the neck joints feedback "
                   "from the robot.";
    }
    neckValues = controlHelper()->jointEncoders();
}

void HeadRetargeting::Impl::initializeNeckJointsSmoother(const unsigned m_actuatedDOFs,
                                                         const double m_dT,
                                                         const double smoothingTime,
                                                         const yarp::sig::Vector jointsInitialValue)
{
    m_NeckJointsPreparationSmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_actuatedDOFs, m_dT, smoothingTime);
    m_NeckJointsPreparationSmoother->init(jointsInitialValue);
}
void HeadRetargeting::Impl::getNeckJointsRefSmoothedValues(yarp::sig::Vector& smoothedJointValues)
{
    m_NeckJointsPreparationSmoother->computeNextValues(m_preparationJointReferenceValues);
    smoothedJointValues = m_NeckJointsPreparationSmoother->getPos();
}
