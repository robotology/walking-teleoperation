/**
 * @file RobotSkin.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

// teleoperation
#include <RobotSkin.hpp>
#include <Utils.hpp>

using namespace HapticGlove;

RobotSkin::RobotSkin(){};

bool RobotSkin::configure(const yarp::os::Searchable& config,
                          const std::string& name,
                          const bool& rightHand)
{
    m_rightHand = rightHand;
    m_logPrefix = "RobotInterface::";
    m_logPrefix += m_rightHand ? "RightHand:: " : "LeftHand:: ";

    m_noFingers = 5;

    std::vector<std::string> humanFingerNameList;

    if (!YarpHelper::getVectorFromSearchable(config, "human_finger_list", humanFingerNameList))
    {
        yError() << m_logPrefix << "unable to get human_finger_list from the config file.";
        return false;
    }
    for (const auto& finger : humanFingerNameList)
    {
        fingertipTactileData fingerdata;
        fingerdata.fingerName = finger;
        std::vector<double> tactileInfo; // it should be positive

        if (!YarpHelper::getVectorFromSearchable(
                config, fingerdata.fingerName + "_tactile_info", tactileInfo))
        {
            yError() << m_logPrefix << "unable to get " << fingerdata.fingerName + "_tactile_info"
                     << "from the config file.";
            return false;
        }
        if (tactileInfo.size() != 3)
        {
            yError() << m_logPrefix << "tactile senor indices for "
                     << fingerdata.fingerName + "_tactile_indices"
                     << "size should be 3, but it is not.";
            return false;
        }
        bool check = (std::round(tactileInfo[0]) >= 0) && (std::round(tactileInfo[1]) > 0)
                     && (std::round(tactileInfo[0]) <= std::round(tactileInfo[1]));

        if (!check)
        {
            yError() << m_logPrefix << "tactile senor indices for "
                     << fingerdata.fingerName + "_tactile_indices"
                     << "should be non-negative.";
            return false;
        }
        if (!(tactileInfo[2] >= 0))
        {
            yError() << m_logPrefix << "the contact threshold should be non-negative.";
            return false;
        }

        fingerdata.indexStart = std::round(tactileInfo[0]);
        fingerdata.indexEnd = std::round(tactileInfo[1]);
        fingerdata.noTactileSensors = fingerdata.indexEnd - fingerdata.indexStart + 1;
        fingerdata.rawTactileData.resize(fingerdata.noTactileSensors);
        fingerdata.calibratedTactileData.resize(fingerdata.noTactileSensors);
        fingerdata.contactThreshold = tactileInfo[2];

        m_fingersTactileData.push_back(fingerdata);
    }
    for (const auto& finger : m_fingersTactileData)
        finger.printInfo();

    return true;
}

void RobotSkin::setRawTactileFeedbacks(const std::vector<double>& rawTactileFeedbacks)
{
    for (auto& finger : m_fingersTactileData)
    {
        for (size_t i = 0; i < finger.noTactileSensors; i++)
        {
            finger.rawTactileData[i] = rawTactileFeedbacks[i + finger.indexStart];

            // crop the data to be sure they are in the range of 0-256
            finger.calibratedTactileData[i] = std::max(
                std::min(finger.rawTactileData[i], finger.maxTactileValue), finger.minTactileValue);

            // crop the data to be sure they are always less than equal to `no load value`
            finger.calibratedTactileData[i]
                = std::min(finger.rawTactileData[i], finger.noLoadValue);

            // normalize the data such that:
            // range: [0,1] ; 0: no load, 1: max load
            finger.calibratedTactileData[i]
                = 1 - double(finger.rawTactileData[i]) / finger.noLoadValue;
        }
    }
}

bool RobotSkin::getFingertipTactileFeedbacks(const size_t fingertipIndex,
                                             std::vector<double>& skinData)
{
    skinData = m_fingersTactileData[fingertipIndex].calibratedTactileData;
    return true;
}

bool RobotSkin::getFingertipMaxTactileFeedback(std::vector<double>& fingertipPressure)
{
    if (fingertipPressure.size() != m_noFingers)
    {
        fingertipPressure.resize(m_noFingers);
    }

    for (size_t i = 0; i < m_noFingers; i++)
    {
        fingertipPressure[i] = m_fingersTactileData[i].maxTactileFeedbackValue();
    }

    return true;
}

void RobotSkin::areFingersInContact(std::vector<bool>& fingersIncontact)
{
    if (fingersIncontact.size() != m_noFingers)
    {
        fingersIncontact.resize(m_noFingers, false);
    }

    for (size_t i = 0; i < m_noFingers; i++)
    {
        fingersIncontact[i] = m_fingersTactileData[i].maxTactileFeedbackValue()
                              > m_fingersTactileData[i].contactThreshold;
    }
}

void RobotSkin::contactStrength(std::vector<double>& fingersContactStrength)
{
    std::vector<bool> fingersIncontact;
    this->areFingersInContact(fingersIncontact);

    if (fingersContactStrength.size() != m_noFingers)
    {
        fingersContactStrength.resize(m_noFingers, false);
    }

    for (size_t i = 0; i < m_noFingers; i++)
    {
        fingersContactStrength[i]
            = 100.0 * (fingersIncontact[i] ? m_fingersTactileData[i].maxTactileFeedbackValue() : 0);
    }
}
