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

    m_robotActuatedJointNameList = robotActuatedJointNameList;
    m_robotActuatedAxisNameList = robotActuatedAxisNameList;
    m_humanJointNameList = humanJointNameList;

    m_logPrefix = "Retargeting::";
}

bool Retargeting::configure(const yarp::os::Searchable& config,
                            const std::string& name,
                            const bool& rightHand)
{
    m_logPrefix += rightHand ? "RightHand:: " : "LeftHand:: ";

    std::vector<std::string> robotAllAxisNames;
    std::vector<std::string> robotActuatedAxisNames;

    yarp::os::Value* robotAllAxisNamesYarp;
    yarp::os::Value* robotActuatedAxisNamesYarp;

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

    if (!config.check("axis_list", robotActuatedAxisNamesYarp))
    {
        yError() << m_logPrefix << "unable to find axis_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(robotActuatedAxisNamesYarp, robotActuatedAxisNames))
    {
        yError() << m_logPrefix
                 << "unable to convert axis_list list into a "
                    "vector of strings.";
        return false;
    }

    std::vector<double> gainValueError;
    if (!YarpHelper::getVectorFromSearchable(config, "gainValueError", gainValueError))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading gainValueError "
                    "vector of the hand.";
        return false;
    }
    this->getCustomSetIndecies(
        robotAllAxisNames, robotActuatedAxisNames, gainValueError, m_gainTotalError);

    std::vector<double> gainVelocityError;
    if (!YarpHelper::getVectorFromSearchable(config, "gainVelocityError", gainVelocityError))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading gainVelocityError "
                    "vector of the hand.";
        return false;
    }

    this->getCustomSetIndecies(
        robotAllAxisNames, robotActuatedAxisNames, gainVelocityError, m_gainVelocityError);

    if (!YarpHelper::getVectorFromSearchable(config, "gainVibrotactile", m_gainVibrotactile))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading gainVibrotactile "
                    "vector of the hand.";
        return false;
    }

    if (!YarpHelper::getVectorFromSearchable(
            config, "human_to_robot_joint_anlges_scaling", m_retargetingScaling))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "human_to_robot_joint_anlges_scaling vector of the hand.";
        return false;
    }

    if (!YarpHelper::getVectorFromSearchable(
            config, "human_to_robot_joint_anlges_bias", m_retargetingBias))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "human_to_robot_joint_anlges_bias vector of the hand.";
        return false;
    }

    if (!YarpHelper::getVectorFromSearchable(
            config, "joints_min_boundary_all", m_robotJointsRangeMin))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "joints_min_boundary_all vector of the hand.";
        return false;
    }

    if (!YarpHelper::getVectorFromSearchable(
            config, "joints_max_boundary_all", m_robotJointsRangeMax))
    {
        yError() << m_logPrefix
                 << "initialization failed while reading "
                    "joints_max_boundary vector of the hand.";
        return false;
    }
    if (m_robotJointsRangeMin.size() != m_robotJointsRangeMax.size())
    {
        yError() << m_logPrefix
                 << "the size of the m_robotJointsRangeMin and "
                    "m_robotJointsRangeMax are not equal: "
                 << m_robotJointsRangeMin.size() << m_robotJointsRangeMax.size();
        return false;
    }

    for (size_t i = 0; i < m_robotJointsRangeMin.size(); i++)
    {
        m_robotJointsRangeMin[i] = m_robotJointsRangeMin[i] * M_PI / 180.0;
        m_robotJointsRangeMax[i] = m_robotJointsRangeMax[i] * M_PI / 180.0;
    }

    // Get human and robot joint list and find the mapping between them
    mapFromHuman2Robot(m_humanJointNameList, m_robotActuatedJointNameList, m_humanToRobotMap);

    m_robotRefJointAngles.resize(m_robotActuatedJointNameList.size(), 0.0);

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

    m_fingerForceFeedback.resize(humanFingersList.size(), 0.0);
    m_fingerVibrotactileFeedback.resize(humanFingersList.size(), 0.0);

    for (std::vector<std::string>::iterator it = humanFingersList.begin();
         it != humanFingersList.end();
         ++it)
    {
        size_t indexFinger = it - humanFingersList.begin();

        yarp::os::Value* axisFingerListYarp;
        std::vector<std::string> axisFingerList;
        if (!config.check(*it, axisFingerListYarp))
        {
            yError() << m_logPrefix << "unable to find " << *it << " into config file.";
            return false;
        }
        if (!YarpHelper::yarpListToStringVector(axisFingerListYarp, axisFingerList))
        {
            yError() << m_logPrefix << "unable to convert " << *it
                     << " list into a "
                        "vector of strings.";
            return false;
        }
        std::vector<size_t> relatedAxes;

        for (std::vector<std::string>::iterator it_axis = axisFingerList.begin();
             it_axis != axisFingerList.end();
             ++it_axis)
        {

            auto elementAxis = std::find(std::begin(m_robotActuatedAxisNameList),
                                         std::end(m_robotActuatedAxisNameList),
                                         *it_axis);
            if (elementAxis != std::end(m_robotActuatedAxisNameList))
            {
                size_t indexAxis = elementAxis - m_robotActuatedAxisNameList.begin();
                relatedAxes.push_back(indexAxis);
            }
        }
        m_fingerAxesMap.insert(std::pair<size_t, std::vector<size_t>>(indexFinger, relatedAxes));
    }

    yInfo() << m_logPrefix << "m_robotJointsRangeMax [rad]: " << m_robotJointsRangeMax;
    yInfo() << m_logPrefix << "m_robotJointsRangeMin [rad]: " << m_robotJointsRangeMin;
    yInfo() << m_logPrefix << "m_gainValueError: " << m_gainTotalError;
    yInfo() << m_logPrefix << "m_gainVelocityError: " << m_gainVelocityError;
    yInfo() << m_logPrefix << "m_gainVibrotactile: " << m_gainVibrotactile;
    yInfo() << m_logPrefix << "m_retargetingScaling: " << m_retargetingScaling;
    yInfo() << m_logPrefix << "m_retargetingBias: " << m_retargetingBias;
    for (const auto& i : m_fingerAxesMap)
        yInfo() << m_logPrefix << "m_fingerAxesMap: " << i.first << " :: " << i.second;

    return true;
}

bool Retargeting::retargetHumanMotionToRobot(const std::vector<double>& humanJointAngles)
{

    if (humanJointAngles.size() != m_humanJointNameList.size())
    {
        yError() << m_logPrefix
                 << "the size of human joint name and "
                    "angles are different."
                 << humanJointAngles.size() << m_humanJointNameList.size();
        return false;
    }

    for (size_t i = 0; i < m_robotActuatedJointNameList.size(); ++i)
    {
        // needs to be updated this function and add as an unordered map
        m_robotRefJointAngles[i]
            = m_retargetingScaling[m_humanToRobotMap[i]] * humanJointAngles[m_humanToRobotMap[i]]
              + m_retargetingBias[m_humanToRobotMap[i]];
        // TO CHECK
        // saturate the references
        m_robotRefJointAngles[i]
            = std::max(m_robotRefJointAngles[i], m_robotJointsRangeMin[m_humanToRobotMap[i]]);
        m_robotRefJointAngles[i]
            = std::min(m_robotRefJointAngles[i], m_robotJointsRangeMax[m_humanToRobotMap[i]]);
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
            // related actuated axis index
            m_fingerForceFeedback[i.first]
                += m_gainTotalError[j]
                   * (axisValueError[j] + m_gainVelocityError[j] * axisVelocityError[j]);
        }
    }
    return true;
}

bool Retargeting::retargetVibroTactileFeedbackFromRobotToHuman()
{

    for (size_t i = 0; i < m_fingerForceFeedback.size(); i++)
    {
        m_fingerVibrotactileFeedback[i] = m_fingerForceFeedback[i] * m_gainVibrotactile[i];
    }
    return true;
}

bool Retargeting::retargetHapticFeedbackFromRobotToHuman(
    const std::vector<double>& axisValueError, const std::vector<double>& axisVelocityError)
{

    this->retargetForceFeedbackFromRobotToHuman(axisValueError, axisVelocityError);
    this->retargetVibroTactileFeedbackFromRobotToHuman();

    return true;
}

bool Retargeting::getRobotJointReferences(std::vector<double>& robotJointReference)
{
    robotJointReference = m_robotRefJointAngles;
    return true;
}

bool Retargeting::getForceFeedbackToHuman(std::vector<double>& forceFeedbackList)
{
    forceFeedbackList = m_fingerForceFeedback;
    return true;
}

bool Retargeting::getVibroTactileFeedbackToHuman(std::vector<double>& buzzFeedbackList)
{
    buzzFeedbackList = m_fingerVibrotactileFeedback;
    return true;
}

bool Retargeting::mapFromHuman2Robot(std::vector<std::string> humanListName,
                                     std::vector<std::string> robotListNames,
                                     std::vector<unsigned>& humanToRobotMap)
{
    if (!humanToRobotMap.empty())
    {
        humanToRobotMap.clear();
    }

    bool foundMatch = false;
    for (unsigned i = 0; i < robotListNames.size(); i++)
    {
        for (unsigned j = 0; j < humanListName.size(); j++)
        {

            if (robotListNames[i] == humanListName[j])
            {
                foundMatch = true;
                humanToRobotMap.push_back(j);
                break;
            }
        }
        if (!foundMatch)
        {
            yError() << "[Retargeting::mapFromHuman2Robot] not found match for: "
                     << robotListNames[i] << " , " << i;
            return false;
        }
        foundMatch = false;
    }

    yInfo() << "*** mapped joint names:  human --> robot ****";
    for (size_t i = 0; i < robotListNames.size(); i++)
    {
        yInfo() << "(" << i << ", " << humanToRobotMap[i] << "): " << robotListNames[i] << " , "
                << humanListName[(humanToRobotMap[i])];
    }

    return true;
}

bool Retargeting::getCustomSetIndecies(const std::vector<std::string>& allListName,
                                       const std::vector<std::string>& customListNames,
                                       const std::vector<double>& allListVector,
                                       std::vector<double>& customListVector)
{
    customListVector.clear();
    if (allListName.empty())
    {
        yInfo() << "[Retargeting::getCustomSetIndecies] all list name is empty.";
        return false;
    }

    bool foundMatch = false;
    for (unsigned i = 0; i < customListNames.size(); i++)
    {
        for (unsigned j = 0; j < allListName.size(); j++)
        {

            if (customListNames[i] == allListName[j])
            {
                foundMatch = true;
                customListVector.push_back(allListVector[j]);
                break;
            }
        }
        if (!foundMatch)
        {
            yError() << "[Retargeting::getCustomSetIndecies] not found match for: "
                     << customListNames[i] << " , " << i;
            return false;
        }
        foundMatch = false;
    }

    if (customListNames.size() != customListVector.size())
    {
        yError() << "[Retargeting::getCustomSetIndecies] customListName and customListVector "
                    "should have similar size";
        return false;
    }

    yInfo() << "*** custom List Vector:****";
    for (size_t i = 0; i < customListNames.size(); i++)
    {
        yInfo() << " (" << i << ", " << customListNames[i] << "): " << customListVector[i];
    }

    return true;
}

bool Retargeting::computeJointAngleRetargetingParams(
    const std::vector<double>& humanHandJointRangeMin,
    const std::vector<double>& humanHandJointRangeMax)
{
    yInfo() << "Retargeting::computeJointAngleRetargetingParams";
    yInfo() << m_robotJointsRangeMax.size() << m_robotJointsRangeMin.size();
    yInfo() << humanHandJointRangeMax.size() << humanHandJointRangeMin.size();
    yInfo() << m_retargetingScaling.size() << m_retargetingBias.size();

    yInfo() << "m_robotJointsRangeMax" << m_robotJointsRangeMax;
    yInfo() << "m_robotJointsRangeMin" << m_robotJointsRangeMin;

    yInfo() << "humanHandJointRangeMax" << humanHandJointRangeMax;
    yInfo() << "humanHandJointRangeMin" << humanHandJointRangeMin;

    yInfo() << "m_retargetingScaling" << m_retargetingScaling;
    yInfo() << "m_retargetingBias" << m_retargetingBias;

    for (size_t i = 0; i < m_retargetingScaling.size(); i++)
    {
        if (m_retargetingScaling[i] >= 0)
        {
            m_retargetingScaling[i] = (m_robotJointsRangeMax[i] - m_robotJointsRangeMin[i])
                                      / (humanHandJointRangeMax[i] - humanHandJointRangeMin[i]);
        } else
        {
            // when the axis of human and robot joint motions are inverse:
            m_retargetingScaling[i] = -1.0 * (m_robotJointsRangeMax[i] - m_robotJointsRangeMin[i])
                                      / (humanHandJointRangeMax[i] - humanHandJointRangeMin[i]);
        }

        m_retargetingBias[i] = (m_robotJointsRangeMax[i] + m_robotJointsRangeMin[i]) / 2.0
                               - m_retargetingScaling[i]
                                     * (humanHandJointRangeMax[i] + humanHandJointRangeMin[i])
                                     / 2.0;
    }
    yInfo() << "[Retargeting::computeJointAngleRetargetingParams] m_retargetingScaling: "
            << m_retargetingScaling;
    yInfo() << "[Retargeting::computeJointAngleRetargetingParams] m_retargetingBias: "
            << m_retargetingBias;

    return true;
}
