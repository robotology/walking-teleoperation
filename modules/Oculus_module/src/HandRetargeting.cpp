/**
 * @file HandRetargeting.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

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

    if (!YarpHelper::getDoubleFromSearchable(config, "scalingFactor", m_scalingFactor))
    {
        yError() << "[HandRetargeting::configure] Unable to find the hands smoothing time";
        return false;
    }

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

void HandRetargeting::setHandTransform(const yarp::sig::Matrix& handTransformation)
{
     iDynTree::toiDynTree(handTransformation, m_oculusInertial_T_handOculusFrame);
}

void HandRetargeting::evaluateDesiredHandPose(yarp::sig::Vector& handPose)
{
    m_teleopRobotFrame_T_handRobotFrame = m_teleopRobotFrame_T_teleopFrame
                                                 * m_oculusInertial_T_teleopFrame.inverse()
                                                 * m_oculusInertial_T_handOculusFrame
                                                 * m_handOculusFrame_T_handRobotFrame;

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
