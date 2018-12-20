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
#include <iDynTree/yarp/YARPConversions.h>

#include <HandRetargeting.hpp>
#include <Utils.hpp>

bool HandRetargeting::configure(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if (config.isNull())
    {
        yError() << "[configure] Empty configuration for hand retargeting.";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(config, "scalingFactor", m_scalingFactor))
    {
        yError() << "[configure] Unable to find the hands smoothing time";
        return false;
    }

    m_rootFixedRotated_T_rootFixed.setRotation(
        iDynTree::Rotation::RPY(0, 0, iDynTree::deg2rad(180)));
    m_rootFixedRotated_T_rootFixed.setPosition(iDynTree::Position::Zero());

    return true;
}

void HandRetargeting::setPlayerOrientation(const double& playerOrientation)
{
    m_root_T_rootFixedRotated.setPosition(iDynTree::Position::Zero());
    m_root_T_rootFixedRotated.setRotation(iDynTree::Rotation::RPY(0, 0, playerOrientation));
}

void HandRetargeting::setHandTransform(const yarp::sig::Matrix& handTransformation)
{
    iDynTree::toiDynTree(handTransformation, m_rootFixed_T_handOculus);
}

void HandRetargeting::evaluateHandToRootLinkTransform(yarp::sig::Vector& handPose)
{
    iDynTree::Transform root_T_hand;
    root_T_hand
        = m_root_T_rootFixedRotated * m_rootFixedRotated_T_rootFixed * m_rootFixed_T_handOculus;

    iDynTree::Vector3 handOrientation, handPosition;
    handOrientation = root_T_hand.getRotation().asRPY();
    handPosition = root_T_hand.getPosition();
    iDynTree::toEigen(handPosition) = m_scalingFactor * iDynTree::toEigen(handPosition);

    handPose.clear();
    handPose.push_back(handPosition(0));
    handPose.push_back(handPosition(1));
    handPose.push_back(handPosition(2));
    handPose.push_back(handOrientation(0));
    handPose.push_back(handOrientation(1));
    handPose.push_back(handOrientation(2));
}
