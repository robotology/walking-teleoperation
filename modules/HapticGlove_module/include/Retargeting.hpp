/**
 * @file Retargeting.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef RETARGETING_HPP
#define RETARGETING_HPP

// std
#include <map>
#include <vector>

// yarp
#include <yarp/os/Searchable.h>

// https://stackoverflow.com/questions/3386861/converting-a-variable-name-to-a-string-in-c
#define VAR_TO_STR(Variable) (void(Variable), #Variable)

namespace HapticGlove
{
class Retargeting;
} // namespace HapticGlove

/**
 * Retargeting Class useful to implement the policy for the retargeting human motions to the robot
 * motions and providing haptic feedback to the user.
 */
class HapticGlove::Retargeting
{
    // member variables
    std::string m_logPrefix;

    size_t m_numAllAxis; /**< the number of all available axis of the robot hand, regardless of
                            actuated ones */
    size_t m_numActuatedAxis; /**< the number of actuated (i.e., used) axis of the robot
                            hand, regardless of actuated ones */
    size_t m_numFingers; /**< number of human hand fingers */

    size_t m_numActuatedJoints; /**< the number of actuated (i.e., used) joints of the robot hand,
                            regardless of actuated ones */
    size_t m_numAllJoints; /**< the number of all available joints of the robot hand,
                           regardless of actuated ones */

    double m_axisContactThreshold; /**< the threshold on the axis value errors in order to be
                                  considered the axis is in contact */

    std::vector<double>
        m_gainTotalError; /**< each element of this vector is multiplied to the
                           error for each motor/axis value error to compute the
                           force feedback associated with each axis, size: actuated axis */
    std::vector<double>
        m_gainVelocityError; /**< each element of this vector is multiplied to the
                              error for each motor/axis velocity error to compute
                              the force feedback associated with each axis, size: actuated axis*/
    std::vector<double>
        m_gainVibrotactile; /**<each element of this vector is multiplied to the force feedback to
                               compute the vibrotactile feedback associated with each finger, size:
                               number of fingers */

    std::vector<double> m_axisValueErrors; /**< the error of the actuated axes values [rad]*/

    std::vector<double> m_axisVelocityErrors; /**< the error of the actuated axes velocities [rad]*/

    std::vector<double>
        m_retargetingScaling; /**< the scale term used to map human joint values to
                               the corresponding robot joint motions, size: actuated joints */
    std::vector<double>
        m_retargetingBias; /**< the bias term used to map human joint vallues to the
                               corresponding robot joint motions, size: actuated joints */

    std::vector<double> m_robotJointsRangeMax; /**< the maximum value robot joints can have */

    std::vector<double> m_robotJointsRangeMin; /**< the minimum value robot joints can have */

    std::vector<double> m_robotAxesMinLimit; /**< the minimum value robot axes can have */

    std::vector<double> m_robotAxesMaxLimit; /**< the maximum value robot axes can have */

    std::vector<std::string> m_robotActuatedAxisNames; /**< name of the robot actuator that has been
                                                          used in teleoperation*/

    std::map<size_t, size_t>
        m_robotToHumanJointIndicesMap; /**< a map from the robot actuated joint index to the
                                          associated human joint index*/
    std::vector<std::string> m_humanJointNames; /**< the name of the all human joints */

    std::vector<std::string> m_robotActuatedJointNames; /**< the name of the all robot joints that
                                                           are used in teleoperation*/

    std::vector<double> m_humanJointAngles; /**< values of the human joints */
    std::vector<double> m_robotRefJointAngles; /**< the reference values for the actuated robot
                                                  joints to follow (the retargeted values)*/
    std::vector<double>
        m_fingerForceFeedback; /**< values of force feedback for the human fingers */
    std::vector<double>
        m_fingerVibrotactileFeedback; /**< values of vibrotactile feedback for the human fingers */

    std::map<size_t, std::vector<size_t>>
        m_fingerAxesMap; /**< a map showing for each finger which axis of robot is relevant*/

    bool m_useSemanticMap = false; /**< a flag that defines if the robot to human joint map is done using semantic map*/
    // methods

    /**
     * get the custom set of vectors
     * @param allListName the full vector names
     * @param customListNames the custm members of the vector names
     * @param allListVector the full vector values
     * @param customListVector the custm members of the vector values
     * @return true/false in case of success/failure
     */
    bool getCustomSetIndices(const std::vector<std::string>& allListName,
                             const std::vector<std::string>& customListNames,
                             const std::vector<double>& allListVector,
                             std::vector<double>& customListVector);

    /**
     * compute the retargeting force feedback to the human hand fingers
     * @param axisValueError errors on the robot axes values
     * @param axisVelocityError errors on the robot axes velocities
     * @return true/false in case of success/failure
     */
    bool retargetForceFeedbackFromRobotToHuman(const std::vector<double>& axisValueError,
                                               const std::vector<double>& axisVelocityError);

    /**
     * compute the retargeting vibrotactile feedback to the human hand fingers using kinesthetic
     * data
     * @return true/false in case of success/failure
     */
    bool retargetKinestheticVibrotactileFeedbackFromRobotToHuman();

    /**
     * compute the retargeting vibrotactile feedback to the human hand fingers using skin data
     * @return true/false in case of success/failure
     */
    bool retargetSkinVibrotactileFeedbackFromRobotToHuman();

    /**
     * compute the retargeting vibrotactile feedback to the human hand fingers using both skin and
     * kinesthetic data
     * @return true/false in case of success/failure
     */
    bool retargetVibrotactileFeedbackFromRobotToHuman();

    /**
     * find the semantic map from the robot actuated joint names to the human joints
     * @param humanJointNames human joint names
     * @param robotJointNames robot actuated joint names
     * @param robotToHumanMap a map from the indices of the actuated robot joints to the indices of
     * the human joints
     * @return true/false in case of success/failure
     */
    bool getSemanticMapFromRobotToHuman(const std::vector<std::string>& humanJointNames,
                                        const std::vector<std::string>& robotJointNames,
                                        std::map<size_t, size_t>& robotToHumanMap);

public:
    /**
     * Constructor
     * @param robotActuatedJointNameList the robot actuated (used) joint names
     * @param robotActuatedAxisNameList the robot actuated (used) joint names
     * hand
     * @param humanJointNameList the names of the human joints
     */
    Retargeting(const std::vector<std::string>& robotActuatedJointNameList,
                const std::vector<std::string>& robotActuatedAxisNameList,
                const std::vector<std::string>& humanJointNameList);

    /**
     * Configure the retargeting class
     * @param config configuration options
     * @param name name of the robot
     * @param rightHand if true the right hand is used
     * @return true/false in case of success/failure
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Retarget the human joint motion to the corresponding actuated robot joint motion
     * @param humanJointAngles the measured human joint angles [rad]
     * @return true/false in case of success/failure
     */
    bool retargetHumanMotionToRobot(const std::vector<double>& humanJointAngles);

    /**
     * compute the retargeting haptic (force and vibrotactile)feedback to the human hand fingers
     * @param axisValueRef reference robot axes values
     * @param axisVelocityRef reference robot axes velocities
     * @param axisValueFb feedback robot axes values
     * @param axisVelocityFb feedback robot axes velocities
     * @return true/false in case of success/failure
     */
    bool retargetHapticFeedbackFromRobotToHumanUsingKinestheticData(
        const std::vector<double>& axisValueRef,
        const std::vector<double>& axisVelocityRef,
        const std::vector<double>& axisValueFb,
        const std::vector<double>& axisVelocityFb);

    /**
     * compute the retargeting haptic (force and vibrotactile)feedback to the human hand fingers
     * @param areFingersSkinWorking true for each finger if its skin is working
     * @param areFingersSkinInContact true for each finger if it is in contact
     * @param skinContactStrength skin contact strength
     * @return true/false in case of success/failure
     */
    bool retargetHapticFeedbackFromRobotToHumanUsingSkinData(
        const std::vector<bool>& areFingersSkinWorking,
        const std::vector<bool>& areFingersSkinInContact,
        const std::vector<double>& skinContactStrength);

    /**
     * get the robot actuated joint references
     * @param robotJointReferences robot actuated joint references
     * @return true/false in case of success/failure
     */
    bool getRobotJointReferences(std::vector<double>& robotJointReferences) const;

    /**
     * get the desired force feedback to the human
     * @param forceFeedbacks the desired force feedback to the human
     * @return true/false in case of success/failure
     */
    bool getForceFeedbackToHuman(std::vector<double>& forceFeedbacks) const;

    /**
     * get the desired vibrotactile feedback to the human
     * @param vibrotactileFeedbacks the desired vibrotactile feedback to the human
     * @return true/false in case of success/failure
     */
    bool getVibrotactileFeedbackToHuman(std::vector<double>& vibrotactileFeedbacks) const;

    /**
     * get the axes errors
     * @param axisValueErrors the axis value errors
     * @param axisVelocityErrors the axis velocity errors
     * @return true/false in case of success/failure
     */
    bool getAxisError(std::vector<double>& axisValueErrors,
                      std::vector<double>& axisVelocityErrors) const;

    /**
     * compute the paramters for the linear configuration space retargeting
     * @param humanHandJointRangeMin the minimum range human joint angles can reach
     * @param humanHandJointRangeMax the maximum range human joint angles can reach
     * @return true/false in case of success/failure
     */
    bool computeJointAngleRetargetingParams(const std::vector<double>& humanHandJointRangeMin,
                                            const std::vector<double>& humanHandJointRangeMax);
    /**
     * Set the axis limits of the robot actuated axes
     * @param robotAxisMinLimit the minimum range robot axis angles can reach (radian)
     * @param robotAxisMaxLimit the maximum range robot axis angles can reach (radian)
     * @return true/false in case of success/failure
     */
    bool setRobotAxisLimits(const std::vector<double>& robotAxisMinLimit,
                            const std::vector<double>& robotAxisMaxLimit);

    /**
     * Set the axis limits of the robot actuated axes
     * @param robotJointMinLimit the minimum range robot axis angles can reach
     * @param robotJointMaxLimit the maximum range robot axis angles can reach
     * @return true/false in case of success/failure
     */
    bool setRobotJointLimits(const std::vector<double>& robotJointMinLimit,
                             const std::vector<double>& robotJointMaxLimit);

    /**
     * close the retageting class
     */
    bool close();
};

#endif // RETARGETING_HPP
