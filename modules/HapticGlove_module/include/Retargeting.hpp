/**
 * @file Retargeting.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef RETARGETING_H
#define RETARGETING_H

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

class HapticGlove::Retargeting
{
    std::string m_logPrefix;

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

    std::vector<double>
        m_retargetingScaling; /**< the scale term used to map human joint values to
                               the corresponding robot joint motions, size: actuated joints */
    std::vector<double>
        m_retargetingBias; /**< the bias term used to map human joint vallues to the
                               corresponding robot joint motions, size: actuated joints */

    std::vector<double> m_robotJointsRangeMax; /**< the maximum value robot joints can have */
    std::vector<double> m_robotJointsRangeMin; /**< the minimum value robot joints can have */

    size_t m_numAllAxis; /**< the number of all available axis of the robot hand, regardless of
                            actuated ones */
    size_t m_numActuatedAxis; /**< the number of actuated (i.e., used) axis of the robot
                            hand, regardless of actuated ones */
    size_t m_numFingers; /**< number of human hand fingers */
    size_t m_numActuatedJoints; /**< the number of actuated (i.e., used) joints of the robot hand,
                            regardless of actuated ones */
    size_t m_numAllJoints; /**< the number of all available joints of the robot hand,
                           regardless of actuated ones */

    std::vector<std::string> m_robotActuatedAxisNames;

    std::map<size_t, size_t> m_robotToHumanJointIndicesMap; /**< comment here */
    std::vector<std::string> m_humanJointNames;
    std::vector<std::string> m_robotActuatedJointNames; /**< comment here */

    std::vector<double> m_humanJointAngles;
    std::vector<double> m_robotRefJointAngles;
    std::vector<double> m_fingerForceFeedback;
    std::vector<double> m_fingerVibrotactileFeedback;

    std::map<size_t, std::vector<size_t>>
        m_fingerAxesMap; /**< a map showing for each finger which axis of robot is relevant*/

public:
    /**
     * Constructor
     * @param numAllAxis the number of all available axis/motors of the robot hand
     * @param numActuatedAxis the number of all actuated (used) axis/motors of the robot hand
     * @param
     * @param
     */
    Retargeting(const std::vector<std::string>& robotActuatedJointNameList,
                const std::vector<std::string>& robotActuatedAxisNameList,
                const std::vector<std::string>& humanJointNameList);

    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    bool retargetHumanMotionToRobot(const std::vector<double>& humanJointAngles);

    bool retargetForceFeedbackFromRobotToHuman(const std::vector<double>& axisValueError,
                                               const std::vector<double>& axisVelocityError);

    bool retargetVibrotactileFeedbackFromRobotToHuman();

    bool retargetHapticFeedbackFromRobotToHuman(const std::vector<double>& axisValueError,
                                                const std::vector<double>& axisVelocityError);

    bool semanticMapFromRobotTHuman(const std::vector<std::string>& humanJointNames,
                                    const std::vector<std::string>& robotJointNames,
                                    std::map<size_t, size_t>& robotToHumanMap);

    bool getRobotJointReferences(std::vector<double>& robotJointReference);

    bool getForceFeedbackToHuman(std::vector<double>& forceFeedbackList);

    bool getVibroTactileFeedbackToHuman(std::vector<double>& buzzFeedbackList);

    bool getCustomSetIndices(const std::vector<std::string>& allListName,
                             const std::vector<std::string>& customListNames,
                             const std::vector<double>& allListVector,
                             std::vector<double>& customListVector);

    bool computeJointAngleRetargetingParams(const std::vector<double>& humanHandJointRangeMin,
                                            const std::vector<double>& humanHandJointRangeMax);
};

#endif // RETARGETING_H
