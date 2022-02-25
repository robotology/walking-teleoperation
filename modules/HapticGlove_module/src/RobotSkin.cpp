/**
 * @file RobotSkin.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

// std
#include <math.h>

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

    std::vector<std::string> robotFingerNameList;

    if (!YarpHelper::getVectorFromSearchable(config, "robot_finger_list", robotFingerNameList))
    {
        yError() << m_logPrefix << "unable to get human_finger_list from the config file.";
        return false;
    }

    m_tactileWorkingThreshold
        = config.check("tactileWorkingThreshold ", yarp::os::Value(0.0001)).asDouble();

    m_noFingers = robotFingerNameList.size();
    m_totalNoTactile = 0;

    m_areFingersInContact.resize(m_noFingers, false);
    m_areTactileSensorsWorking.resize(m_noFingers, false);

    m_fingersVibrotactileFeedback.resize(m_noFingers, 0.0);
    m_fingersContactStrength.resize(m_noFingers, 0.0);

    for (const auto& finger : robotFingerNameList)
    {
        FingertipTactileData fingerdata;
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
        fingerdata.tactileData.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.biasTactileSensor.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.stdTactileSensor.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.calibratedTactileData.resize(fingerdata.noTactileSensors, 0.0);

        fingerdata.contactThresholdValue = tactileInfo[2];
        fingerdata.vibrotactileGain = tactileInfo[3];

        fingerdata.collectedTactileData.resize(Eigen::NoChange, fingerdata.noTactileSensors);

        m_fingersTactileData.push_back(fingerdata);
        m_totalNoTactile += fingerdata.noTactileSensors;
    }
    yInfo() << m_logPrefix << "====== Skin Information ======";
    yInfo() << m_logPrefix << "number of fingers: " << m_noFingers;
    yInfo() << m_logPrefix << "number of tactile sensors: " << m_totalNoTactile;
    yInfo() << m_logPrefix << "tactile working threshold: " << m_tactileWorkingThreshold;

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

            // crop the data to be sure they are in the range of 0-255
            finger.tactileData[i] = std::max(
                std::min(finger.rawTactileData[i], finger.maxTactileValue), finger.minTactileValue);

            // normalize the data such that:
            // range: [0,1] ; 0: no load, 1: max load
            finger.tactileData[i] = 1.0 - (finger.tactileData[i] / finger.maxTactileValue);

            std::transform(finger.tactileData.begin(),
                           finger.tactileData.end(),
                           finger.biasTactileSensor.begin(),
                           finger.calibratedTactileData.begin(),
                           std::minus<double>());
        }
    }
}

bool RobotSkin::collectSkinDataForCalibration()
{

    for (auto& data : m_fingersTactileData)
    {

        Eigen::VectorXd tactileData = CtrlHelper::toEigenVector(data.tactileData);

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
    int counter = 0;
    for (auto& data : m_fingersTactileData)
    {
        bool tactileSenorsWork = false;
        for (size_t i = 0; i < data.noTactileSensors; i++)
        {
            Eigen::VectorXd vec = data.collectedTactileData.col(i);
            data.biasTactileSensor[i] = vec.mean();
            data.stdTactileSensor[i]
                = std::sqrt(((vec.array() - vec.mean()).square().sum()) / vec.size());
            // if a tactile senors does not work its std is zero
            // normally either all or none of the tactile sensors of a fingertip work
            // so if at least one tactile sensor works, the skin works
            tactileSenorsWork |= (data.stdTactileSensor[i] > m_tactileWorkingThreshold);
        }
        yInfo() << m_logPrefix << data.fingerName << ": mean of tactile sensors"
                << data.biasTactileSensor;
        yInfo() << m_logPrefix << data.fingerName << ": standard deviation of tactile sensors"
                << data.stdTactileSensor;
        if (counter > m_noFingers)
        {
            yError() << m_logPrefix
                     << " the size of m_fingersTactileData is more the number of fingers: "
                     << counter;
            return false;
        }
        m_areTactileSensorsWorking[counter] = tactileSenorsWork;
        counter++;
    }
    yInfo() << m_logPrefix << "are tactile senors working: " << m_areTactileSensorsWorking;
    return true;
}

bool RobotSkin::getFingertipTactileFeedbacks(const size_t fingertipIndex,
                                             std::vector<double>& skinData)
{
    skinData = m_fingersTactileData[fingertipIndex].tactileData;
    return true;
}

void RobotSkin::updateTactileFeedbacks(const std::vector<double>& rawTactileFeedbacks)
{
    this->setRawTactileFeedbacks(rawTactileFeedbacks);

    this->computeAreFingersInContact();

    this->computeMaxContactStrength();

    this->computeVibrotactileFeedback();
}

void RobotSkin::computeAreFingersInContact()
{
    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_areFingersInContact[i] = m_fingersTactileData[i].maxTactileFeedbackValue()
                                   > m_fingersTactileData[i].contactThreshold();
    }
}

void RobotSkin::computeMaxContactStrength()
{

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersContactStrength[i]
            = (m_areFingersInContact[i] ? m_fingersTactileData[i].maxTactileFeedbackValue() : 0);
    }
}

void RobotSkin::computeVibrotactileFeedback()
{
    for (size_t i = 0; i < m_noFingers; i++)
    {
        double x = 100.0 * m_fingersTactileData[i].vibrotactileGain * m_fingersContactStrength[i];

        m_fingersVibrotactileFeedback[i]
            = 15.0 * std::log(2 * std::pow(x, 0.7) + 1) + 0.5 * std::pow(x, 1.1);
    }
}

void RobotSkin::getContactStrength(std::vector<double>& fingersContactStrength)
{
    fingersContactStrength = m_fingersContactStrength;
}

void RobotSkin::getVibrotactileFeedback(std::vector<double>& fingersVibrotactileFeedback)
{
    fingersVibrotactileFeedback = m_fingersVibrotactileFeedback;
}

void RobotSkin::areFingersInContact(std::vector<bool>& areFingersIncontact)
{
    areFingersIncontact = m_areFingersInContact;
}

void RobotSkin::doesTactileSensorsWork(std::vector<bool>& tactileSensorsAreWorking)
{
    tactileSensorsAreWorking = m_areTactileSensorsWorking;
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
        std::copy(data.tactileData.begin(),
                  data.tactileData.end(),
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

const size_t RobotSkin::getNumOfTactileFeedbacks()
{
    return m_totalNoTactile;
}
