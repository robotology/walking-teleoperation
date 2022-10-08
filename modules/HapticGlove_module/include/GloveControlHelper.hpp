/**
 * @file GloveControlHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef GLOVE_CONTROL_HELPER_HPP
#define GLOVE_CONTROL_HELPER_HPP

#include <Eigen/Dense>

// std
#include <memory>
#include <vector>

// Wearable library
#include "GloveWearable.hpp"

// YARP
#include <yarp/os/Searchable.h>

namespace HapticGlove
{
class GloveControlHelper;
} // namespace HapticGlove

/**
 * GloveControlHelper is an helper class for controlling the glove.
 */
class HapticGlove::GloveControlHelper
{

    std::string m_logPrefix;

    const size_t m_numForceFeedback; /**< Number of the motors to produce force feedback to the
                         human*/
    const size_t m_numVibrotactileFeedback; /**< Number of the vibrotactile to produce vibrotactile
                                      feedback to the human*/
    const size_t m_numFingers; /**< Number of the fingers of the glove/human */
    const size_t m_numHandJoints; /**< Number of the joints of the human hand*/
    const double m_maxForceFeedback; /**< Max force feedback the glove at each fingertip can apply
                                     to the human [N]*/

    bool m_isRightHand; /**< true if the glove is the right hand*/

    std::vector<int> m_desiredForceValues; /**< Desired force feedback values; range: [0, 100]. */
    std::vector<int>
        m_desiredVibrotactileValues; /**< Desired vibrotactile feedback values; range: [0, 100] */
    std::vector<double>
        m_JointsValues; /**< human hand joints angles [rad];
                            -- fingers: from thumb to pinky
                            -- joints: from  oppose/abduction , proximal, middle, distal*/
    std::vector<std::string> m_humanJointNameList; /**< The names of the human hand joints
                            -- fingers: from thumb to pinky
                            -- joints: from  oppose/abduction , proximal, middle, distal*/
    std::vector<std::string>
        m_humanFingerNameList; /**< The names of the human hand fingers, from thumb to pinky*/
    std::vector<double> m_jointRangeMin; /**< Min value for every human hand joint, computed at the
                                            end of the preparation phase*/
    std::vector<double> m_jointRangeMax; /**< Max value for every human hand joint, computed at the
                                            end of the preparation phase*/

    std::vector<double> m_jointRangeMaxDelta;
    std::vector<double> m_jointRangeMinDelta;

    std::unique_ptr<GloveWearableImpl>
        m_pImp; /**< Sense glove wearable interface impelemntation. */

public:
    /**
     * Constructor
     */
    GloveControlHelper();

    /**
     * Configure the helper
     * @param config configuration options
     * @param name name of the robot
     * @param rightHand if true the right hand is used
     * @return true/false in case of success/failure
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Get the measured fingertip poses of all the fingers
     * @param measuredValues measured fingertip poses matrix (Number of fingers x 7)
     * @return true / false in case of success / failure
     */
    bool getFingertipPoses(Eigen::MatrixXd& measuredValues);

    /**
     * Get the measured joint values
     * @param jointAngles measured joint values
     * @return true/false in case of success/failure
     */
    bool getHandJointAngles(std::vector<double>& jointAngles);

    /**
     * Set the desired force feedback reference to the user fingertip
     * @param desiredValue desired force feedback values
     * @return true/false in case of success/failure
     */
    bool setFingertipForceFeedbackReferences(const std::vector<double>& desiredValue);

    /**
     * Set the vibro-tactile feedback references to the user
     * @param desiredValue desired vibro-tactile feedback values
     * @return true/false in case of success/failure
     */
    bool setFingertipVibrotactileFeedbackReferences(
        const std::vector<double>& desiredValue); // to change data type

    /**
     * Set the number of vibro-tactile reference
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success/failure
     */
    bool setPalmVibrotactileFeedbackReference(const int& desiredValue);

    /**
     * stop the haptic feedback (force feedback, fingertip vibrotactile feedback, palm vibrotactile
     * feedback) to the user
     * @return  true/false in case of success/failure
     */
    bool stopHapticFeedback();

    /**
     * Stop the vibro-tactile feedback to the user
     * @return true/false in case of success / failure
     */
    bool stopPalmVibrotactileFeedback();

    /**
     * Stop the vibro-tactile feedback to the user fingertip
     * @return true/false in case of success / failure
     */
    bool stopVibrotactileFeedback();

    /**
     * Stop the force feedback to the user fingertip
     * @return true/false in case of success / failure
     */
    bool stopForceFeedback();

    /**
     * Setup glove helper for running
     * @return true/false in case of success/failure
     */
    bool setupGlove();

    /**
     * Get the name of a joint given the index of the joint in the list
     * @param i the index of the joint to be retrieved
     * @param jointName the name of joint associated with the i'th joint
     * @return  true/false in case of success/failure
     */
    bool getHumanHandJointName(const size_t i, std::string& jointName) const;

    /**
     * Get the list of the human hand joint names
     * @param jointNameList human hand joints name list
     * @return true/false in case of success/failure
     */
    bool getHumanHandJointsNames(std::vector<std::string>& jointNameList) const;

    /**
     * Get the name of a finger given the index of the finger in the list
     * @param i the index of the finger to be retrieved
     * @param fingerName the name of finger associated with the i'th finger
     * @return  true/false in case of success/failure
     */
    bool getHumanHandFingerName(const size_t i, std::string& fingerName) const;

    /**
     * Get the list of the human finger names
     * @param fingerNameList human hand fingers name list
     * @return true/false in case of success/failure
     */
    bool getHumanHandFingerNames(std::vector<std::string>& fingerNameList) const;

    /**
     * Find the range of the motion of the human finger joints of the hand
     * @return true/false in case of success/failure
     */
    bool findHumanMotionRange();

    /**
     * Get the range of the motion of the human finger joints of the hand at the end of the
     * preparation time
     * @param jointRangeMin human hand finger joints lower bound values retrieved from user during
     * prepation time
     * @param jointRangeMax human hand finger joints upper bound values retrieved from user during
     * prepation time
     * @return true/false in case of success/failure
     */
    bool getHumanFingerJointsMotionRange(std::vector<double>& jointRangeMin,
                                         std::vector<double>& jointRangeMax) const;

    /**
     * Get the hand palm rotation from the IMU data
     * @param data glove IMU data attached to the human hand palm (order x y z w)
     * @return true/false in case of success/failure
     */
    bool getHandPalmRotation(std::vector<double>& data) const;

    /**
     * Get the number of human/glove fingers
     * @return the number of fingers
     */
    const size_t getNumOfFingers() const;

    /**
     * Get the number of human/glove hand joints
     * @return the number of hand joints
     */
    const size_t getNumOfHandJoints() const;

    /**
     * Get the number of glove vibrotactile feedbacks
     * @return the number of vibrotactile feedbacks
     */
    const size_t getNumOfVibrotactileFeedbacks() const;

    /**
     * Get the number of glove force feedbacks
     * @return the number of force feedbacks
     */
    const size_t getNumOfForceFeedback() const;

    /**
     * Close the helper
     * @return true/false in case of success/failure
     */
    bool close();
};

#endif // GLOVE_CONTROL_HELPER_HPP
