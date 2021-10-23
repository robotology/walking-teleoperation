/**
 * @file Retargeting.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

// std
#include <algorithm>
#include <math.h>

// yarp
#include <yarp/os/Value.h>

// walking-teleoperation
#include <Retargeting.hpp>
#include <Utils.hpp>

using namespace HapticGlove;

Retargeting::Retargeting(const std::vector<std::string>& robotActuatedJointNameList,
                         const std::vector<std::string>& robotActuatedAxisNameList,
                         const std::vector<std::string>& humanJointNameList)
{

    m_robotActuatedJointNames = robotActuatedJointNameList;
    m_robotActuatedAxisNames = robotActuatedAxisNameList;
    m_humanJointNames = humanJointNameList;

    m_logPrefix = "Retargeting::";
}

bool Retargeting::configure(const yarp::os::Searchable& config,
                            const std::string& name,
                            const bool& rightHand)
{
    m_logPrefix += rightHand ? "RightHand:: " : "LeftHand:: ";

    std::vector<std::string> robotAllAxisNames;
    std::vector<std::string> robotAllJointNames;

    yarp::os::Value* robotAllAxisNamesYarp;
    yarp::os::Value* robotAllJointNamesYarp;

    if (!config.check("all_axis_list", robotAllAxisNamesYarp))
    {
        yError() << m_logPrefix << "unable to find all_axis_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(robotAllAxisNamesYarp, robotAllAxisNames))
    {
        yError() << m_logPrefix
                 << "unable to convert all_axis_list list into a "
                    "vector of strings.";
        return false;
    }

    if (!config.check("joint_list", robotAllJointNamesYarp))
    {
        yError() << m_logPrefix << "unable to find axis_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(robotAllJointNamesYarp, robotAllJointNames))
    {
        yError() << m_logPrefix
                 << "unable to convert axis_list list into a "
                    "vector of strings.";
        return false;
    }

    m_numAllAxis = robotAllAxisNames.size();
    m_numActuatedAxis = m_robotActuatedAxisNames.size();
    m_numActuatedJoints = m_robotActuatedJointNames.size();
    m_numAllJoints = robotAllJointNames.size();

    // get the total gain used for haptic feedback based on impedance control
    std::vector<double> gainTotalError;
    if (!YarpHelper::getVectorFromSearchable(config, "gainTotalError", gainTotalError))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading gainValueError "
                    "vector of the hand.";
        return false;
    }

    if (!YarpHelper::checkSizeOfVector<double>(
            gainTotalError, m_numAllAxis, VAR_TO_STR(gainTotalError), m_logPrefix))
    {
        return false;
    }

    if (!this->getCustomSetIndices(
            robotAllAxisNames, m_robotActuatedAxisNames, gainTotalError, m_gainTotalError))
    {
        yError() << m_logPrefix << "cannot get the custom set for" << VAR_TO_STR(gainTotalError);
        return false;
    }

    // get the veclocity gain used for haptic feedback based on impedance control
    std::vector<double> gainVelocityError;
    if (!YarpHelper::getVectorFromSearchable(config, "gainVelocityError", gainVelocityError))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading gainVelocityError "
                    "vector of the hand.";
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(
            gainVelocityError, m_numAllAxis, VAR_TO_STR(gainVelocityError), m_logPrefix))
    {
        return false;
    }

    if (!this->getCustomSetIndices(
            robotAllAxisNames, m_robotActuatedAxisNames, gainVelocityError, m_gainVelocityError))
    {
        yError() << m_logPrefix << "cannot get the custom set for" << VAR_TO_STR(gainVelocityError);
        return false;
    }

    // get the robot joints retargeting scales (the values later will be updated, the sign of the
    // values are important)
    std::vector<double> retargetingScaling;
    if (!YarpHelper::getVectorFromSearchable(
            config, "human_to_robot_joint_anlges_scaling", retargetingScaling))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "human_to_robot_joint_anlges_scaling vector of the hand.";
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(
            retargetingScaling, m_numAllJoints, VAR_TO_STR(retargetingScaling), m_logPrefix))
    {
        return false;
    }
    if (!this->getCustomSetIndices(robotAllJointNames,
                                   m_robotActuatedJointNames,
                                   retargetingScaling,
                                   m_retargetingScaling))
    {
        yError() << m_logPrefix << "cannot get the custom set for"
                 << VAR_TO_STR(retargetingScaling);
        return false;
    }

    // get the robot joints retargeting bias (the values later will be updated)
    std::vector<double> retargetingBias;
    if (!YarpHelper::getVectorFromSearchable(
            config, "human_to_robot_joint_anlges_bias", retargetingBias))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "human_to_robot_joint_anlges_bias vector of the hand.";
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(
            retargetingBias, m_numAllJoints, VAR_TO_STR(retargetingBias), m_logPrefix))
    {
        return false;
    }
    if (!this->getCustomSetIndices(
            robotAllJointNames, m_robotActuatedJointNames, retargetingBias, m_retargetingBias))
    {
        yError() << m_logPrefix << "cannot get the custom set for" << VAR_TO_STR(retargetingBias);
        return false;
    }

    // get the robot joints minimum values
    std::vector<double> robotJointsRangeMin;
    if (!YarpHelper::getVectorFromSearchable(
            config, "joints_min_boundary_all", robotJointsRangeMin))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "joints_min_boundary_all vector of the hand.";
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(
            robotJointsRangeMin, m_numAllJoints, VAR_TO_STR(robotJointsRangeMin), m_logPrefix))
    {
        return false;
    }
    if (!this->getCustomSetIndices(robotAllJointNames,
                                   m_robotActuatedJointNames,
                                   robotJointsRangeMin,
                                   m_robotJointsRangeMin))
    {
        yError() << m_logPrefix << "cannot get the custom set for"
                 << VAR_TO_STR(robotJointsRangeMin);
        return false;
    }

    // get the robot joints maximum values
    std::vector<double> robotJointsRangeMax;
    if (!YarpHelper::getVectorFromSearchable(
            config, "joints_max_boundary_all", robotJointsRangeMax))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "joints_max_boundary vector of the hand.";
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(
            robotJointsRangeMax, m_numAllJoints, VAR_TO_STR(robotJointsRangeMax), m_logPrefix))
    {
        return false;
    }
    if (!this->getCustomSetIndices(robotAllJointNames,
                                   m_robotActuatedJointNames,
                                   robotJointsRangeMax,
                                   m_robotJointsRangeMax))
    {
        yError() << m_logPrefix << "cannot get the custom set for"
                 << VAR_TO_STR(robotJointsRangeMax);
        return false;
    }

    for (size_t i = 0; i < m_numActuatedJoints; i++)
    {
        m_robotJointsRangeMin[i] = m_robotJointsRangeMin[i] * M_PI / 180.0;
        m_robotJointsRangeMax[i] = m_robotJointsRangeMax[i] * M_PI / 180.0;
    }

    // get human and robot joint list and find the mapping between them
    if (!this->semanticMapFromRobotTHuman(
            m_humanJointNames, m_robotActuatedJointNames, m_robotToHumanJointIndicesMap))
    {
        yError() << m_logPrefix
                 << "unable to find the map from robot actuated joints to the human joints";
        return false;
    }

    // find the human finger names
    yarp::os::Value* humanFingersListYarp;
    std::vector<std::string> humanFingersList;
    if (!config.check("human_finger_list", humanFingersListYarp))
    {
        yError() << m_logPrefix << "unable to find human_finger_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(humanFingersListYarp, humanFingersList))
    {
        yError() << m_logPrefix
                 << "unable to convert human_finger_list list into a "
                    "vector of strings.";
        return false;
    }
    m_numFingers = humanFingersList.size();

    // find all the associated and actuated axis to each human finger used for the haptic feedback
    for (std::vector<std::string>::iterator fingerName = humanFingersList.begin();
         fingerName != humanFingersList.end();
         ++fingerName)
    {
        size_t indexOfFinger = fingerName - humanFingersList.begin();

        yarp::os::Value* axisFingerListYarp;
        std::vector<std::string> axisFingerList;
        if (!config.check(*fingerName, axisFingerListYarp))
        {
            yError() << m_logPrefix << "unable to find " << *fingerName << " into config file.";
            return false;
        }
        if (!YarpHelper::yarpListToStringVector(axisFingerListYarp, axisFingerList))
        {
            yError() << m_logPrefix << "unable to convert " << *fingerName
                     << " list into a "
                        "vector of strings.";
            return false;
        }
        std::vector<size_t> relatedAxes;

        for (std::vector<std::string>::iterator it_axis = axisFingerList.begin();
             it_axis != axisFingerList.end();
             ++it_axis)
        {

            auto elementAxis = std::find(
                std::begin(m_robotActuatedAxisNames), std::end(m_robotActuatedAxisNames), *it_axis);
            if (elementAxis != std::end(m_robotActuatedAxisNames))
            {
                size_t indexOfAxis = elementAxis - m_robotActuatedAxisNames.begin();
                relatedAxes.push_back(indexOfAxis);
            }
        }
        m_fingerAxesMap.insert(std::pair<size_t, std::vector<size_t>>(indexOfFinger, relatedAxes));
    }

    if (!YarpHelper::getVectorFromSearchable(config, "gainVibrotactile", m_gainVibrotactile))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading gainVibrotactile "
                    "vector of the hand.";
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(
            m_gainVibrotactile, m_numFingers, VAR_TO_STR(m_gainVibrotactile), m_logPrefix))
    {
        return false;
    }

    // initialize the vectors
    m_robotRefJointAngles.resize(m_numActuatedJoints, 0.0);
    m_fingerForceFeedback.resize(m_numFingers, 0.0);
    m_fingerVibrotactileFeedback.resize(m_numFingers, 0.0);

    // print information
    yInfo() << m_logPrefix << "m_robotJointsRangeMax [rad]: " << m_robotJointsRangeMax;
    yInfo() << m_logPrefix << "m_robotJointsRangeMin [rad]: " << m_robotJointsRangeMin;
    yInfo() << m_logPrefix << "m_gainValueError: " << m_gainTotalError;
    yInfo() << m_logPrefix << "m_gainVelocityError: " << m_gainVelocityError;
    yInfo() << m_logPrefix << "m_gainVibrotactile: " << m_gainVibrotactile;
    yInfo() << m_logPrefix << "m_retargetingScaling: " << m_retargetingScaling;
    yInfo() << m_logPrefix << "m_retargetingBias: " << m_retargetingBias;
    for (const auto& i : m_fingerAxesMap)
        yInfo() << m_logPrefix << "m_fingerAxesMap: " << i.first << " :: " << i.second;
    for (const auto& i : m_robotToHumanJointIndicesMap)
        yInfo() << m_logPrefix << "m_humanToRobotMap: " << i.first << " :: " << i.second;

    yInfo() << m_logPrefix << "configuration is done.";
    return true;
}

bool Retargeting::retargetHumanMotionToRobot(const std::vector<double>& humanJointAngles)
{

    if (!YarpHelper::checkSizeOfVector<double>(
            humanJointAngles, m_humanJointNames.size(), VAR_TO_STR(humanJointAngles), m_logPrefix))
    {
        return false;
    }

    for (size_t i = 0; i < m_numActuatedJoints; ++i)
    {
        size_t idx = m_robotToHumanJointIndicesMap[i];

        // find the desired robot joint angle
        m_robotRefJointAngles[i]
            = m_retargetingScaling[i] * humanJointAngles[idx] + m_retargetingBias[idx];

        // saturate the references
        m_robotRefJointAngles[i] = std::max(m_robotRefJointAngles[i], m_robotJointsRangeMin[i]);
        m_robotRefJointAngles[i] = std::min(m_robotRefJointAngles[i], m_robotJointsRangeMax[i]);
    }

    return true;
}

bool Retargeting::retargetForceFeedbackFromRobotToHuman(
    const std::vector<double>& axisValueError, const std::vector<double>& axisVelocityError)
{
    for (const auto& i : m_fingerAxesMap)
    {
        m_fingerForceFeedback[i.first] = 0.0;
        for (const auto& j : i.second)
        {
            // use the relevant actuated axis for each finger to compute the force feedback
            m_fingerForceFeedback[i.first]
                += m_gainTotalError[j]
                   * (axisValueError[j] + m_gainVelocityError[j] * axisVelocityError[j]);
        }
    }
    return true;
}

bool Retargeting::retargetVibrotactileFeedbackFromRobotToHuman()
{

    for (size_t i = 0; i < m_numFingers; i++)
    {
        m_fingerVibrotactileFeedback[i] = m_fingerForceFeedback[i] * m_gainVibrotactile[i];
    }
    return true;
}

bool Retargeting::retargetHapticFeedbackFromRobotToHuman(
    const std::vector<double>& axisValueError, const std::vector<double>& axisVelocityError)
{
    // check the input vector sizes
    if (!YarpHelper::checkSizeOfVector<double>(
            axisValueError, m_numActuatedAxis, VAR_TO_STR(axisValueError), m_logPrefix))
    {
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(
            axisVelocityError, m_numActuatedAxis, VAR_TO_STR(axisVelocityError), m_logPrefix))
    {
        return false;
    }

    if (!this->retargetForceFeedbackFromRobotToHuman(axisValueError, axisVelocityError))
    {
        yError() << m_logPrefix << "cannot compute force feedback from robot to the human.";
        return false;
    }
    if (!this->retargetVibrotactileFeedbackFromRobotToHuman())
    {
        yError() << m_logPrefix << "cannot compute vibrotactile feedback from robot to the human.";
        return false;
    }

    return true;
}

bool Retargeting::getRobotJointReferences(std::vector<double>& robotJointReferences)
{
    robotJointReferences = m_robotRefJointAngles;
    return true;
}

bool Retargeting::getForceFeedbackToHuman(std::vector<double>& forceFeedbacks)
{
    forceFeedbacks = m_fingerForceFeedback;
    return true;
}

bool Retargeting::getVibrotactileFeedbackToHuman(std::vector<double>& vibrotactileFeedbacks)
{
    vibrotactileFeedbacks = m_fingerVibrotactileFeedback;
    return true;
}

bool Retargeting::semanticMapFromRobotTHuman(const std::vector<std::string>& humanJointNames,
                                             const std::vector<std::string>& robotJointNames,
                                             std::map<size_t, size_t>& robotToHumanMap)
{
    if (!robotToHumanMap.empty())
    {
        robotToHumanMap.clear();
    }

    for (unsigned i = 0; i < robotJointNames.size(); i++)
    {
        auto index
            = std::find(std::begin(humanJointNames), std::end(humanJointNames), robotJointNames[i]);

        if (index == std::end(humanJointNames))
        {
            yError() << m_logPrefix << "not found match for: " << robotJointNames[i]
                     << " , element: " << i << ".";
            return false;
        }
        size_t elementNumber = index - robotJointNames.begin();

        robotToHumanMap.insert(std::pair<size_t, size_t>(i, elementNumber));
    }
    if (robotToHumanMap.size() != robotJointNames.size())
    {
        yError() << m_logPrefix << "robotToHumanMap size is not equal to the robotJointNames size:"
                 << "robotToHumanMap.size(): " << robotToHumanMap.size()
                 << " , robotJointNames.size(): " << robotJointNames.size();
        return false;
    }

    return true;
}

bool Retargeting::getCustomSetIndices(const std::vector<std::string>& allListName,
                                      const std::vector<std::string>& customListNames,
                                      const std::vector<double>& allListVector,
                                      std::vector<double>& customListVector)
{
    // check the sizes
    customListVector.clear();
    if (allListName.empty())
    {
        yError() << m_logPrefix << " allListName is empty.";
        return false;
    }
    if (allListVector.empty())
    {
        yError() << m_logPrefix << " allListVector is empty.";
        return false;
    }
    if (allListVector.size() != allListName.size())
    {
        yError() << m_logPrefix << " allListVector and allListName do not have similar sizes..";
        return false;
    }

    // find the custom vector
    for (unsigned i = 0; i < customListNames.size(); i++)
    {
        auto index = std::find(std::begin(allListName), std::end(allListName), customListNames[i]);

        if (index == std::end(allListName))
        {
            yError() << m_logPrefix << "cannot find a match for: " << customListNames[i]
                     << " , element: " << i << " in the allListName.";
            return false;
        }
        size_t element = index - allListName.begin();

        customListVector.push_back(allListVector[element]);
    }

    // one last final check
    if (customListNames.size() != customListVector.size())
    {
        yError() << m_logPrefix
                 << "customListName and customListVector "
                    "do not have similar sizes.";
        return false;
    }

    return true;
}

bool Retargeting::computeJointAngleRetargetingParams(
    const std::vector<double>& humanHandJointRangeMin,
    const std::vector<double>& humanHandJointRangeMax)
{
    // check the sizes of the vectors
    if (!YarpHelper::checkSizeOfVector<double>(humanHandJointRangeMin,
                                               m_humanJointNames.size(),
                                               VAR_TO_STR(humanHandJointRangeMin),
                                               m_logPrefix))
    {
        return false;
    }
    if (!YarpHelper::checkSizeOfVector<double>(humanHandJointRangeMax,
                                               m_humanJointNames.size(),
                                               VAR_TO_STR(humanHandJointRangeMax),
                                               m_logPrefix))
    {
        return false;
    }

    for (size_t i = 0; i < m_numActuatedJoints; i++)
    {
        const size_t idx = m_robotToHumanJointIndicesMap[i];

        // when the axis of human and robot joint motions are inverse, it is necessary to mutiply by
        // -1.0;
        const double multiplier = (m_retargetingScaling[i] >= 0) ? 1.0 : -1.0;

        m_retargetingScaling[i] = multiplier * (m_robotJointsRangeMax[i] - m_robotJointsRangeMin[i])
                                  / (humanHandJointRangeMax[idx] - humanHandJointRangeMin[idx]);

        m_retargetingBias[i] = (m_robotJointsRangeMax[i] + m_robotJointsRangeMin[i]) / 2.0
                               - m_retargetingScaling[i]
                                     * (humanHandJointRangeMax[idx] + humanHandJointRangeMin[idx])
                                     / 2.0;
    }

    yInfo() << m_logPrefix << "m_retargetingScaling: " << m_retargetingScaling;
    yInfo() << m_logPrefix << "m_retargetingBias: " << m_retargetingBias;

    return true;
}
