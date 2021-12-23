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
    m_logPrefix = "RobotSkin::";
    m_logPrefix += m_rightHand ? "RightHand:: " : "LeftHand:: ";

    std::vector<std::string> humanFingerNameList;

    if (!YarpHelper::getVectorFromSearchable(config, "human_finger_list", humanFingerNameList))
    {
        yError() << m_logPrefix << "unable to get human_finger_list from the config file.";
        return false;
    }
    m_noFingers = humanFingerNameList.size();
    m_totalNoTactile = 0;

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
        if (tactileInfo.size() != 4)
        {
            yError() << m_logPrefix << "tactile senor indices for "
                     << fingerdata.fingerName + "_tactile_indices"
                     << "size should be 4, but it is not.";
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
        fingerdata.vibrotactileGain = tactileInfo[3];

        fingerdata.biasTactileSensor.resize(fingerdata.noTactileSensors);
        fingerdata.stdTactileSensor.resize(fingerdata.noTactileSensors);
        fingerdata.collectedTactileData.resize(Eigen::NoChange, fingerdata.noTactileSensors);

        m_fingersTactileData.push_back(fingerdata);
        m_totalNoTactile += fingerdata.noTactileSensors;
    }
    yInfo() << "===== Skin Information =======";
    yInfo() << m_logPrefix << "number of fingers: " << m_noFingers;
    yInfo() << m_logPrefix << "number of tactile sensors: " << m_totalNoTactile;

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
            //            finger.calibratedTactileData[i]
            //                = std::min(finger.calibratedTactileData[i], finger.noLoadValue);

            // normalize the data such that:
            // range: [0,1] ; 0: no load, 1: max load
            finger.calibratedTactileData[i]
                = 1 - double(finger.calibratedTactileData[i]) / finger.noLoadValue;
        }
    }
}

bool RobotSkin::collectSkinDataForCalibration()
{

    for (auto& data : m_fingersTactileData)
    {

        Eigen::VectorXd tactileData = CtrlHelper::toEigenVector(data.calibratedTactileData);

        if (!CtrlHelper::push_back_row(data.collectedTactileData, tactileData.transpose()))
        {
            yError() << m_logPrefix
                     << "cannot add new axes feedback values to the collected axes data .";
            return false;
        }
    }

    return true;
}

bool RobotSkin::computeCalibrationParamters()
{
    for (auto& data : m_fingersTactileData)
    {
        for (size_t i = 0; i < data.noTactileSensors; i++)
        {
            Eigen::VectorXd vec = data.collectedTactileData.col(i);
            yInfo() << m_logPrefix << data.fingerName
                    << "data size:" << data.collectedTactileData.rows()
                    << data.collectedTactileData.cols() << vec.size();
            data.biasTactileSensor[i] = vec.mean();
            data.stdTactileSensor[i]
                = std::sqrt(((vec.array() - vec.mean()).square().sum()) / vec.size());
        }
        yInfo() << m_logPrefix << data.fingerName << ": mean of tactile sensors"
                << data.biasTactileSensor;
        yInfo() << m_logPrefix << data.fingerName << ": standard deviation of tactile sensors"
                << data.stdTactileSensor;
    }
    return true;
}

bool RobotSkin::getFingertipTactileFeedbacks(const size_t fingertipIndex,
                                             std::vector<double>& skinData)
{
    skinData = m_fingersTactileData[fingertipIndex].calibratedTactileData;
    return true;
}

bool RobotSkin::getSerializedFingertipsTactileFeedbacks(
    std::vector<double>& fingertipsTactileFeedback)
{
    if (fingertipsTactileFeedback.size() != m_totalNoTactile)
    {
        fingertipsTactileFeedback.resize(m_totalNoTactile, 0.0);
    }
    size_t start = 0;
    for (const auto& data : m_fingersTactileData)
    {
        std::copy(data.calibratedTactileData.begin(),
                  data.calibratedTactileData.end(),
                  fingertipsTactileFeedback.begin() + start);
        start += data.noTactileSensors;
    }

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
            = (fingersIncontact[i] ? m_fingersTactileData[i].maxTactileFeedbackValue() : 0);
    }
}

void RobotSkin::vibrotactileFeedback(std::vector<double>& fingersVibrotactileFeedback)
{
    this->contactStrength(fingersVibrotactileFeedback);
    for (size_t i = 0; i < m_noFingers; i++)
    {
        fingersVibrotactileFeedback[i]
            = 100.0 * m_fingersTactileData[i].vibrotactileGain * fingersVibrotactileFeedback[i];
    }
}

const size_t RobotSkin::getNumOfTactileFeedbacks()
{
    return m_totalNoTactile;
}
