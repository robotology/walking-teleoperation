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

#include <TorsoRetargeting.hpp>
#include <Utils.hpp>

iDynTree::Rotation TorsoRetargeting::forwardKinematics(const double& torsoPitch,
                                                       const double& torsoRoll,
                                                       const double& torsoYaw)
{
    iDynTree::Rotation chest_R_root;
    chest_R_root = iDynTree::Rotation::RotY(-torsoPitch) * iDynTree::Rotation::RotX(torsoRoll)
                   * iDynTree::Rotation::RotZ(-torsoYaw);

    return chest_R_root;
}

bool TorsoRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    // check if the configuration file is empty
    if (config.isNull())
    {
        yError() << "[TorsoRetargeting::configure] Empty configuration for torso retargeting.";
        return false;
    }

    m_controlHelper = std::make_unique<RobotControlHelper>();
    if (!m_controlHelper->configure(config, name, true))
    {
        yError() << "[TorsoRetargeting::configure] Unable to configure the torso helper";
        return false;
    }

    return true;
}

bool TorsoRetargeting::move()
{
    yInfo() << "Nothing Implemented!";
    return true;
}
