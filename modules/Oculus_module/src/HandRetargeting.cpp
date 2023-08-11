// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <HandRetargeting.hpp>
#include <Utils.hpp>

bool HandRetargeting::configure(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if (config.isNull())
    {
        yError() << "[HandRetargeting::configure] Empty configuration for hand retargeting.";
        return false;
    }
    double humanHeight, robotArmSpan;
    if (!YarpHelper::getDoubleFromSearchable(config, "humanHeight", humanHeight))
    {
        yError() << "[HandRetargeting::configure] Unable to find the humanHeight parameter.";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(config, "robotArmSpan", robotArmSpan))
    {
        yError() << "[HandRetargeting::configure] Unable to find the robotArmSpan parameter.";
        return false;
    }

    m_scalingFactor = robotArmSpan / humanHeight;
    yInfo()
        << "[HandRetargeting::configure] kinematic scaling factor (= robotArmSpan / humanHeight): "
        << m_scalingFactor;

    // getting the mapping between the robot and the retargeting frames
    iDynTree::Rotation tempRotation;
    if (!iDynTree::parseRotationMatrix(config, "handOculusFrame_R_handRobotFrame", tempRotation))
    {
        yError() << "[HandRetargeting::configure] Unable to find the hands Oculus to hand robot "
                    "rotation matrix";
        return false;
    }

    m_handOculusFrame_T_handRobotFrame.setRotation(tempRotation);
    m_handOculusFrame_T_handRobotFrame.setPosition(iDynTree::Position::Zero());

    if (!iDynTree::parseRotationMatrix(
            config, "teleoperationRobotFrame_R_teleoperationFrame", tempRotation))
    {
        yError() << "[HandRetargeting::configure] Unable to find the teleoperation robot to "
                    "teleoperation rotation matrix";
        return false;
    }

    m_teleopRobotFrame_T_teleopFrame.setRotation(tempRotation);
    m_teleopRobotFrame_T_teleopFrame.setPosition(iDynTree::Position::Zero());

    m_oculusInertial_T_teleopFrame.setPosition(iDynTree::Position::Zero());

    return true;
}

void HandRetargeting::setPlayerOrientation(const double& playerOrientation)
{
    // notice the minus sign is not an error. Indeed the virtualizer angle is positive clockwise
    m_oculusInertial_T_teleopFrame.setRotation(iDynTree::Rotation::RotZ(-playerOrientation));
}

void HandRetargeting::setPlayerPosition(const iDynTree::Position& playerPosition)
{
    m_oculusInertial_T_teleopFrame.setPosition(playerPosition);
}

void HandRetargeting::setHandTransform(const yarp::sig::Matrix& handTransformation)
{
    iDynTree::toiDynTree(handTransformation, m_oculusInertial_T_handOculusFrame);
}

void HandRetargeting::evaluateDesiredHandPose(yarp::sig::Vector& handPose)
{
    m_teleopRobotFrame_T_handRobotFrame
        = m_teleopRobotFrame_T_teleopFrame * m_oculusInertial_T_teleopFrame.inverse()
          * m_oculusInertial_T_handOculusFrame * m_handOculusFrame_T_handRobotFrame;

    // probably we should avoid to use roll pitch and yaw. A possible solution
    // is to use quaternion or directly SE(3).
    iDynTree::Vector3 handOrientation, handPosition;
    handOrientation = m_teleopRobotFrame_T_handRobotFrame.getRotation().asRPY();
    handPosition = m_teleopRobotFrame_T_handRobotFrame.getPosition();
    iDynTree::toEigen(handPosition) = m_scalingFactor * iDynTree::toEigen(handPosition);

    handPose.clear();
    handPose.push_back(handPosition(0));
    handPose.push_back(handPosition(1));
    handPose.push_back(handPosition(2));
    handPose.push_back(handOrientation(0));
    handPose.push_back(handOrientation(1));
    handPose.push_back(handOrientation(2));
}

void HandRetargeting::getHandInfo(std::vector<double>& robotHandposeWrtRobotTel,
                                  std::vector<double>& humanHandposeWrtOculusInertial,
                                  std::vector<double>& humanHandposeWrtHumanTel)
{
    // robot hand pose wrt robot teleoperation frame
    iDynTree::Vector3 handOrientation, handPosition;
    handOrientation = m_teleopRobotFrame_T_handRobotFrame.getRotation().asRPY();
    handPosition = m_teleopRobotFrame_T_handRobotFrame.getPosition();
    iDynTree::toEigen(handPosition) = m_scalingFactor * iDynTree::toEigen(handPosition);

    robotHandposeWrtRobotTel.clear();
    robotHandposeWrtRobotTel.push_back(handPosition(0));
    robotHandposeWrtRobotTel.push_back(handPosition(1));
    robotHandposeWrtRobotTel.push_back(handPosition(2));
    robotHandposeWrtRobotTel.push_back(handOrientation(0));
    robotHandposeWrtRobotTel.push_back(handOrientation(1));
    robotHandposeWrtRobotTel.push_back(handOrientation(2));

    // human hand pose wrt oculus inertial frame
    iDynTree::Vector3 handOrientationInertial, handPositionInertial;
    handOrientationInertial = m_oculusInertial_T_handOculusFrame.getRotation().asRPY();
    handPositionInertial = m_oculusInertial_T_handOculusFrame.getPosition();

    humanHandposeWrtOculusInertial.clear();
    humanHandposeWrtOculusInertial.push_back(handPositionInertial(0));
    humanHandposeWrtOculusInertial.push_back(handPositionInertial(1));
    humanHandposeWrtOculusInertial.push_back(handPositionInertial(2));
    humanHandposeWrtOculusInertial.push_back(handOrientationInertial(0));
    humanHandposeWrtOculusInertial.push_back(handOrientationInertial(1));
    humanHandposeWrtOculusInertial.push_back(handOrientationInertial(2));

    // human hand pose wrt human teleopration frame
    iDynTree::Transform teleopFrame_T_handOculusFrame
        = m_oculusInertial_T_teleopFrame.inverse() * m_oculusInertial_T_handOculusFrame;

    iDynTree::Vector3 handOrientationTeleoperation, handPositionTeleoperation;
    handOrientationTeleoperation = teleopFrame_T_handOculusFrame.getRotation().asRPY();
    handPositionTeleoperation = teleopFrame_T_handOculusFrame.getPosition();

    humanHandposeWrtHumanTel.clear();
    humanHandposeWrtHumanTel.push_back(handPositionTeleoperation(0));
    humanHandposeWrtHumanTel.push_back(handPositionTeleoperation(1));
    humanHandposeWrtHumanTel.push_back(handPositionTeleoperation(2));
    humanHandposeWrtHumanTel.push_back(handOrientationTeleoperation(0));
    humanHandposeWrtHumanTel.push_back(handOrientationTeleoperation(1));
    humanHandposeWrtHumanTel.push_back(handOrientationTeleoperation(2));
}
