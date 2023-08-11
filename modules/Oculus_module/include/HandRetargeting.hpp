// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HAND_RETARGETING_HPP
#define HAND_RETARGETING_HPP

// std
#include <vector>

// YARP
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

// iDynTree
#include <iDynTree/Core/Transform.h>

/**
 * HandRetargeing manages the retargeting of the hand.
 * It's main objectivity is to evaluate the desire hand pose with respect the teleoperation
 * frame.
 */
class HandRetargeting
{
private:
    // In order to understand the transform defined the following frames has to be defined
    // oculusInertial frame: it is the inertial frame of the oculus and it is placed in the
    //                       initial position of the ovrheadset. The z axis points upward while
    //                       x point forward
    // teleop frame: this frame has the same origin of the oculusInertial frame but it is
    //               rigidly attached to the user. In details the oculusInertial frame and
    //               the teleoperation frame differs for a rotation along the Z axis.
    // handOculus frame: frame attached to the oculus joypad.
    // handRobot frame: frame attached to the robot hand.
    // teleopRobot frame: attached to the robot w.r.t the desired handRobot pose is
    //                    evaluated

    /** Transform between the inertial oculus frame and the teleoperation frame (This transform
     * depends on the virtualizer angle) */
    iDynTree::Transform m_oculusInertial_T_teleopFrame;

    /** Transform between the inertial oculus frame and the hand frame */
    iDynTree::Transform m_oculusInertial_T_handOculusFrame;

    /** Mapping between the hand oculus frame and the hand robot frame */
    iDynTree::Transform m_handOculusFrame_T_handRobotFrame;

    /** Mapping between the teleoperation frame and the robot teleoperation frame. (Notice that the
     * robot teleoperation frame is in this specific case the "imu_frame". You can use a different
     * frame but it has to be cooerent with one chosen in th walking-controller) */
    iDynTree::Transform m_teleopRobotFrame_T_teleopFrame;

    /** Desired tranformation between the teleoperation robot frame and the hand robot frame */
    iDynTree::Transform m_teleopRobotFrame_T_handRobotFrame;

    double m_scalingFactor; /**< Scaling factor */

public:
    /**
     * Configure the hand retargeting.
     * @param config reference to a resource finder object.
     * @return true in case of success and false otherwise
     */
    bool configure(const yarp::os::Searchable& config);

    /**
     * Set the player orientation (coming from the virtualizer)
     * @param playerOrientation orientation of the player in radiant
     */
    void setPlayerOrientation(const double& playerOrientation);

    /**
     * Set the player position (coming from the ovr headset)
     * @param playerPosition Position of the player head (teleopartion frame) in meter
     */
    void setPlayerPosition(const iDynTree::Position& playerPosition);

    /**
     * Set the root tho hand transformation
     * @param handTransformation root to hand transformation (root_T_hand)
     */
    void setHandTransform(const yarp::sig::Matrix& handTransformation);

    /**
     * Evaluate the desired hand pose tacking into account the relative transformation between the
     * user and the virtualizer (if it is used)
     * @param handPose desired hand position. This can be directly sent to walking controller.
     */
    void evaluateDesiredHandPose(yarp::sig::Vector& handPose);

    /**
     * Get the hand information
     * user and the virtualizer (if it is used)
     * @param robotHandpose_robotTel robot hand pose wrt to robot teleoperation frame.
     * @param humanHandpose_oculusInertial human hand pose wrt to oculus inertial frame
     * @param humanHandpose_humanTel human hand pose wrt to human teleoperation frame
     */
    void getHandInfo(std::vector<double>& robotHandpose_robotTel,
                     std::vector<double>& humanHandpose_oculusInertial,
                     std::vector<double>& humanHandpose_humanTel);
};

#endif
