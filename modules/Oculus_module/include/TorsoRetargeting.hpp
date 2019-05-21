/**
 * @file HeadRetargeting.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 *          Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef TORSO_RETARGETING_HPP
#define TORSO_RETARGETING_HPP

// std
#include <memory>

// YARP
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/minJerkCtrl.h>

// iDynTree
#include <iDynTree/Core/Rotation.h>

#include <RetargetingController.hpp>

/**
 * Class useful to get torso info for retargeting.
 */
class TorsoRetargeting : public RetargetingController
{
public:
    /**
     * Configure the object.
     * @param config is the reference to a resource finder object.
     * @param name is the name of the robot.
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    /**
     * Evaluate the forward kinematics of the torso
     * Further details on the joints name can be found in
     * http://wiki.icub.org/wiki/ICub_Model_naming_conventions#Joints
     * @param torsoPitch neck pitch angle expressed in radiant
     * @param torsoRoll neck roll angle expressed in radiant
     * @param torsoYaw neck yaw angle expressed in radiant
     * @return rotation matrix of the chest with respect to the root_link frame
     */
    static iDynTree::Rotation
    forwardKinematics(const double& torsoPitch, const double& torsoRoll, const double& torsoYaw);

    bool move() override;
};

#endif
