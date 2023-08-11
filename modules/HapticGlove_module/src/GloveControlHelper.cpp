// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <GloveControlHelper.hpp>
#include <Utils.hpp>
#include <limits>

using namespace HapticGlove;

GloveControlHelper::GloveControlHelper()
    : m_numFingers(5)
    , m_numForceFeedback(5)
    , m_numVibrotactileFeedback(5)
    , m_numHandJoints(20) // abduction joints are ignored
    , m_maxForceFeedback(40) // Netwton
{
    m_logPrefix = "GloveControlHelper::";
}

bool GloveControlHelper::configure(const yarp::os::Searchable& config,
                                   const std::string& name,
                                   const bool& rightHand)
{

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    m_isRightHand = rightHand;

    m_logPrefix += m_isRightHand ? "RightHand:: " : "LeftHand:: ";

    m_desiredVibrotactileValues.resize(m_numVibrotactileFeedback, 0);
    m_desiredForceValues.resize(m_numForceFeedback, 0);

    m_JointsValues.resize(m_numHandJoints, 0.0);

    yarp::os::Value* jointListYarp;
    if (!config.check("human_joint_list", jointListYarp))
    {
        yError() << m_logPrefix << "unable to find human_joint_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(jointListYarp, m_humanJointNameList))
    {
        yError() << m_logPrefix
                 << "unable to convert human_joint_list list into a "
                    "vector of strings.";
        return false;
    }
    if (m_humanJointNameList.size() != m_numHandJoints)
    {
        yError() << m_logPrefix
                 << "the number of joints in the configuration "
                    "file is not equal to the const number of joints";
        return false;
    }
    // fingers
    yarp::os::Value* fingerListYarp;
    if (!config.check("human_finger_list", fingerListYarp))
    {
        yError() << m_logPrefix << "unable to find human_finger_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(fingerListYarp, m_humanFingerNameList))
    {
        yError() << m_logPrefix
                 << "unable to convert human_finger_list list into a "
                    "vector of strings.";
        return false;
    }
    if (m_humanFingerNameList.size() != m_numFingers)
    {
        yError() << m_logPrefix
                 << "the number of hand fingers in the configuration "
                    "file is not equal to the const number of fingers";
        return false;
    }
    m_jointRangeMin.resize(m_humanJointNameList.size(), 0.0);
    m_jointRangeMax.resize(m_humanJointNameList.size(), 0.0);
    m_jointRangeMinDelta.resize(m_humanJointNameList.size(), 0.0);
    m_jointRangeMaxDelta.resize(m_humanJointNameList.size(), 0.0);

    // human motion range minimum delta
    if (config.check("motion_range_min_delta") && config.find("motion_range_min_delta").isList())
    {
        yarp::os::Bottle* jointRangeMinDeltaMap = config.find("motion_range_min_delta").asList();
        for (size_t i = 0; i < jointRangeMinDeltaMap->size(); i++)
        {
            yarp::os::Bottle* jointRangeMinDeltaValue = jointRangeMinDeltaMap->get(i).asList();
            std::string jointName = jointRangeMinDeltaValue->get(0).asString();

            double deltaMinVal =jointRangeMinDeltaValue->get(1).asFloat64() * M_PI / 180; // [rad]

            auto jointElement
                = std::find(std::begin(m_humanJointNameList), std::end(m_humanJointNameList), jointName);
            if (jointElement == std::end(m_humanJointNameList))
            {
                yError() << m_logPrefix << "cannot find the joint " << jointName
                         << "written in `motion_range_min_delta` among the humanFingerNameList.";
                return false;
            }

            m_jointRangeMinDelta[std::distance(m_humanJointNameList.begin(), jointElement)] = deltaMinVal;
        }
    }

    // human motion range maximum delta
    if (config.check("motion_range_max_delta") && config.find("motion_range_max_delta").isList())
    {
        yarp::os::Bottle* jointRangeMaxDeltaMap = config.find("motion_range_max_delta").asList();
        for (size_t i = 0; i < jointRangeMaxDeltaMap->size(); i++)
        {
            yarp::os::Bottle* jointRangeMaxDeltaValue = jointRangeMaxDeltaMap->get(i).asList();
            std::string jointName = jointRangeMaxDeltaValue->get(0).asString();

            double deltaMaxVal =jointRangeMaxDeltaValue->get(1).asFloat64() * M_PI / 180; // [rad]

            auto jointElement
                = std::find(std::begin(m_humanJointNameList), std::end(m_humanJointNameList), jointName);
            if (jointElement == std::end(m_humanJointNameList))
            {
                yError() << m_logPrefix << "cannot find the joint " << jointName
                         << "written in `motion_range_max_delta` among the humanFingerNameList.";
                return false;
            }

            m_jointRangeMaxDelta[std::distance(m_humanJointNameList.begin(), jointElement)] = deltaMaxVal;
        }
    }

    // wearable device
    m_pImp = std::make_unique<GloveWearableImpl>(
        m_numFingers, m_numForceFeedback, m_numVibrotactileFeedback, m_numHandJoints);
    if (!m_pImp->configure(config, name, m_isRightHand))
    {
        yError() << m_logPrefix << "unable to configure the haptic glove wearable device.";
        return false;
    }

    if (!this->setupGlove())
    {
        yError() << m_logPrefix << "cannot setup the glove.";
        return false;
    }
    yInfo() << m_logPrefix << "configuration is done.";

    return true;
}

bool GloveControlHelper::getFingertipPoses(Eigen::MatrixXd& measuredValues)
{
    return m_pImp->getFingertipPoseValues(measuredValues);
}

bool GloveControlHelper::getHandJointAngles(std::vector<double>& jointAngles)
{
    return m_pImp->getJointValues(jointAngles);
}

bool GloveControlHelper::setFingertipForceFeedbackReferences(
    const std::vector<double>& desiredValue)
{
    if (desiredValue.size() != m_numForceFeedback)
    {
        yError() << m_logPrefix
                 << "the size of the input "
                    "desired vecotr ["
                 << desiredValue.size() << "] and the number of haptic force feedbacks [ "
                 << m_numForceFeedback << " ]are not equal.";
        return false;
    }

    for (size_t i = 0; i < m_numForceFeedback; i++)
    {
        m_desiredForceValues[i]
            = (int)std::round(std::max(0.0, std::min(desiredValue[i], m_maxForceFeedback)) * 100
                              / m_maxForceFeedback);
    }

    return m_pImp->setFingertipForceFeedbackValues(m_desiredForceValues);
}

bool GloveControlHelper::setFingertipVibrotactileFeedbackReferences(
    const std::vector<double>& desiredValue)
{

    if (desiredValue.size() != m_numVibrotactileFeedback)
    {
        yError() << m_logPrefix << "the size of the input desired vector [" << desiredValue.size()
                 << "] and the number of haptic force feedbacks [ " << m_numVibrotactileFeedback
                 << " ]are not equal.";
        return false;
    }

    for (size_t i = 0; i < m_numVibrotactileFeedback; i++)
    {
        m_desiredVibrotactileValues[i]
            = (int)std::round(std::max(0.0, std::min(desiredValue[i], 100.0)));
    }

    return m_pImp->setFingertipVibrotactileValues(m_desiredVibrotactileValues);
}
bool GloveControlHelper::stopPalmVibrotactileFeedback()
{
    return m_pImp->setPalmVibrotactileValue(to_underlying(
        HapticGlove::SenseGlove::ThumperCmd::TurnOff)); // to turn off set `124`, for more details
                                                        // check the documentation.
}

bool GloveControlHelper::stopVibrotactileFeedback()
{
    std::fill(m_desiredVibrotactileValues.begin(), m_desiredVibrotactileValues.end(), 0.0);
    return m_pImp->setFingertipVibrotactileValues(m_desiredVibrotactileValues);
}

bool GloveControlHelper::stopForceFeedback()
{
    std::fill(m_desiredForceValues.begin(), m_desiredForceValues.end(), 0.0);
    return m_pImp->setFingertipForceFeedbackValues(m_desiredForceValues);
}

bool GloveControlHelper::stopHapticFeedback()
{

    if (!stopVibrotactileFeedback())
    {
        yError() << m_logPrefix << "Cannot turn off the fingertip vibrotactile feedback";
        return false;
    }
    if (!stopForceFeedback())
    {
        yError() << m_logPrefix << "Cannot turn off the fingertip froce feedback";
        return false;
    }
    if (!stopPalmVibrotactileFeedback())
    {
        yError() << "[GloveControlHelper::stopHapticFeedback] Cannot turn off the palm "
                    "vibrotactile feedback";
        return false;
    }
    return true;
}

const size_t GloveControlHelper::getNumOfVibrotactileFeedbacks() const
{
    return m_numVibrotactileFeedback;
}

const size_t GloveControlHelper::getNumOfForceFeedback() const
{
    return m_numForceFeedback;
}

const size_t GloveControlHelper::getNumOfFingers() const
{
    return m_numFingers;
}

const size_t GloveControlHelper::getNumOfHandJoints() const
{
    return m_numHandJoints;
}

bool GloveControlHelper::close()
{
    yInfo() << m_logPrefix << "trying to close";
    bool ok = true;
    if (!this->stopHapticFeedback())
    {
        yWarning() << m_logPrefix << "unable to close the glove control helper.";
        ok &= false;
    }

    if (!m_pImp->close())
    {
        yWarning() << m_logPrefix << "unable to close the glove wearable implementation.";
        ok &= false;
    }

    yInfo() << m_logPrefix << "closed" << (ok ? "Successfully" : "badly") << ".";

    return ok;
}

bool GloveControlHelper::setupGlove()
{
    std::vector<double> jointAngleList;
    this->getHandJointAngles(jointAngleList);
    for (size_t i = 0; i < m_humanJointNameList.size(); i++)
    {
        m_jointRangeMin[i] = jointAngleList[i];
        m_jointRangeMax[i] = jointAngleList[i];
    }
    return true;
}

bool GloveControlHelper::setPalmVibrotactileFeedbackReference(const int& desiredValue)
{
    return m_pImp->setPalmVibrotactileValue(desiredValue);
}

bool GloveControlHelper::getHumanHandJointName(const size_t i, std::string& jointName) const
{
    if (i >= this->getNumOfHandJoints())
    {
        yError() << m_logPrefix
                 << "The requested i'th joint name is bigger than equal to the number of joints.";
        return false;
    }
    jointName = m_humanJointNameList[i];
    return true;
}

bool GloveControlHelper::getHumanHandJointsNames(std::vector<std::string>& jointNameList) const
{
    if (m_humanJointNameList.size() != this->getNumOfHandJoints())
    {
        yError() << m_logPrefix
                 << "The number of human hand joints and joints name list size are different.";
        return false;
    }
    jointNameList = m_humanJointNameList;
    return true;
}

bool GloveControlHelper::getHumanHandFingerName(const size_t i, std::string& fingerName) const
{
    if (i >= this->getNumOfFingers())
    {
        yError() << m_logPrefix
                 << "The requested i'th finger name is bigger than equal to the number of fingers.";
        return false;
    }
    fingerName = m_humanFingerNameList[i];
    return true;
}

bool GloveControlHelper::getHumanHandFingerNames(std::vector<std::string>& fingerNameList) const
{
    if (m_humanFingerNameList.size() != this->getNumOfFingers())
    {
        yError() << m_logPrefix
                 << "The number of human hand finger and finger name list size are different.";
        return false;
    }
    fingerNameList = m_humanFingerNameList;
    return true;
}

bool GloveControlHelper::findHumanMotionRange()
{
    std::vector<double> jointAngleList;
    this->getHandJointAngles(jointAngleList);

    for (size_t i = 0; i < m_humanJointNameList.size(); i++)
    {
        m_jointRangeMin[i] = std::min(m_jointRangeMin[i], jointAngleList[i]);
        m_jointRangeMax[i] = std::max(m_jointRangeMax[i], jointAngleList[i]);
    }

    std::vector<double> desiredValue(m_numVibrotactileFeedback, 35);
    this->setFingertipVibrotactileFeedbackReferences(desiredValue);
    return true;
}

bool GloveControlHelper::getHumanFingerJointsMotionRange(std::vector<double>& jointRangeMin,
                                                         std::vector<double>& jointRangeMax) const
{
    jointRangeMax = m_jointRangeMax;
    jointRangeMin = m_jointRangeMin;

    // Add deltas on human motion range
    for (size_t i = 0; i < m_humanJointNameList.size(); i++)
    {
        jointRangeMax[i] = jointRangeMax[i] + m_jointRangeMaxDelta[i];
        jointRangeMin[i] = jointRangeMin[i] + m_jointRangeMinDelta[i];
    }

    yInfo() << m_logPrefix << "human joint names:" << m_humanJointNameList;
    yInfo() << m_logPrefix << "human min joint range:" << m_jointRangeMin;
    yInfo() << m_logPrefix << "human max joint range:" << m_jointRangeMax;

    return true;
}

bool GloveControlHelper::getHandPalmRotation(std::vector<double>& data) const
{
    return m_pImp->getPalmImuRotationValues(data);
}
