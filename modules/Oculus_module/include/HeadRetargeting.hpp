/**
 * @file HeadRetargeting.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef HEAD_RETARGETING_HPP
#define HEAD_RETARGETING_HPP

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
 * Class useful to manage the head retargeting.
 */
class HeadRetargeting : public RetargetingController
{
private:
    /** Minimum jerk trajectory smoother for the desired head joints */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_headTrajectorySmoother{nullptr};

    // In order to understand the transform defined the following frames has to be defined
    // oculusInertial frame: it is the inertial frame of the oculus and it is placed in the
    //                       initial position of the ovrheadset. The z axis points upward while
    //                       x point forward
    // teleop frame: this frame has the same origin of the oculusInertial frame but it is
    //               rigidly attached to the user. In details the oculusInertial frame and
    //               the teleoperation frame differs for a rotation along the Z axis.
    // headOculus frame: frame attached to the oculus head.
    iDynTree::Rotation m_oculusInertial_R_teleopFrame;
    iDynTree::Rotation m_oculusInertial_R_headOculus;
    iDynTree::Rotation m_teleopFrame_R_headOculus;

public:
    /**
     * Configure the object.
     * @param config is the reference to a resource finder object.
     * @param name is the name of the robot.
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    /**
     * Set the player orientation
     * @param playerOrientation is the orientation of the player in radiant (positive angle
     * correspond to a clockwise rotation)
     */
    void setPlayerOrientation(const double& playerOrientation);

    /**
     * Set the desired head orientation.
     * @param oculusInertial_T_headOculus is the homogeneous transformation between the oculus
     * inertial frame and the head oculus frame
     */
    void setDesiredHeadOrientation(const yarp::sig::Matrix& oculusInertial_T_headOculus);

    /**
     * Evaluate the inverse kinematics of the head
     * Further details on the joints name can be found in
     * http://wiki.icub.org/wiki/ICub_Model_naming_conventions#Joints
     * @param chest_R_head is the rotation matrix of the head with respect the chest frame
     * @param neckPitch neck pitch angle expressed in radiant
     * @param neckRoll neck roll angle expressed in radiant
     * @param neckYaw neck yaw angle expressed in radiant
     */
    static void inverseKinematics(const iDynTree::Rotation& chest_R_head,
                                  double& neckPitch,
                                  double& neckRoll,
                                  double& neckYaw);

    /**
     * Evaluate the forward kinematics of the head
     * Further details on the joints name can be found in
     * http://wiki.icub.org/wiki/ICub_Model_naming_conventions#Joints
     * @param neckPitch neck pitch angle expressed in radiant
     * @param neckRoll neck roll angle expressed in radiant
     * @param neckYaw neck yaw angle expressed in radiant
     * @return rotation matrix of the head with respect the chest frame
     */
    static iDynTree::Rotation
    forwardKinematics(const double& neckPitch, const double& neckRoll, const double& neckYaw);

    /**
     * Evaluate the neck joints according to the desired head pose
     */
    void evalueNeckJointValues();

    /**
     * Initialize neck joints values at preparation time with Zero value
     * @return intialized if the neck is initialized return true
     */
    bool initializeNeckJointValues();

    /**
     * Get the neck joints values
     * @param neckValues neck joint values vector
     */
    void getNeckJointValues(std::vector<double>& neckValues);

    /**
     * Move the neck joints according to the desired joint values
     * @return true in case of success and false otherwise
     */
    bool move() override;
};

#endif
