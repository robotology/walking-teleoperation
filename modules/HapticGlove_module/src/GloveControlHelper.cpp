/**
 * @file GloveControlHelper.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <limits>

// iDynTree
#include <iDynTree/Core/Utils.h>

#include <GloveControlHelper.hpp>
#include <Utils.hpp>

#include <Wearable/IWear/Sensors/IVirtualJointKinSensor.h>
#include <yarp/dev/PolyDriver.h>

using namespace HapticGlove;

bool GloveControlHelper::configure(const yarp::os::Searchable& config,
                                   const std::string& name,
                                   const bool& rightHand)
{

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    m_numFingers = 5;
    m_numForceFeedback = m_numFingers;
    m_numVibrotactileFeedback = m_numFingers;
    m_numHandJoints = 20;

    m_isRightHand = rightHand;
    m_desiredVibrotactileValues.resize(m_numVibrotactileFeedback, 0);
    m_desiredForceValues.resize(m_numForceFeedback, 0);

    m_JointsValues.resize(m_numHandJoints, 0.0);

    yarp::os::Value* jointListYarp;
    if (!config.check("human_joint_list", jointListYarp))
    {
        yError()
            << "[GloveControlHelper::configure] Unable to find human_joint_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(jointListYarp, m_humanJointNameList))
    {
        yError()
            << "[GloveControlHelper::configure] Unable to convert human_joint_list list into a "
               "vector of strings.";
        return false;
    }

    yarp::os::Value* fingerListYarp;
    if (!config.check("human_finger_list", fingerListYarp))
    {
        yError()
            << "[GloveControlHelper::configure] Unable to find human_finger_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(fingerListYarp, m_humanFingerNameList))
    {
        yError()
            << "[GloveControlHelper::configure] Unable to convert human_finger_list list into a "
               "vector of strings.";
        return false;
    }
    m_jointRangeMin.resize(m_humanJointNameList.size(), 0.0);
    m_jointRangeMax.resize(m_humanJointNameList.size(), 0.0);

    // wearable device

    m_pImp = std::make_unique<GloveWearableImpl>();
    if (!m_pImp->configure(config, name, m_isRightHand))
    {
        yError() << "unable to configure the haptic glove wearable device.";
        return false;
    }

    return true;
}

bool GloveControlHelper::getFingertipPoses(Eigen::MatrixXd& measuredValues)
{
    // to implement

    return true;
}

bool GloveControlHelper::getHandJointAngles(std::vector<double>& jointAngles)
{

    if (!m_pImp->getFingersJointValues(jointAngles))
    {
        yError() << "getHandJointsAngles: error in geting new human joint angles.";
        return false;
    }
    return true;
}

bool GloveControlHelper::setFingertipForceFeedbackReferences(const yarp::sig::Vector& desiredValue)
{
    if (desiredValue.size() != m_numForceFeedback)
    {
        yError() << "[GloveControlHelper::setFingersForceReference] the size of the input "
                    "desired vecotr ["
                 << desiredValue.size() << "] and the number of haptic force feedbacks [ "
                 << m_numForceFeedback << " ]are not equal.";
        return false;
    }

    for (size_t i = 0; i < m_numForceFeedback; i++)
    {
        m_desiredForceValues[i] = desiredValue(i);
    }

    m_pImp->setFingersForceFeedbackValues(m_desiredForceValues);

    return true;
}

bool GloveControlHelper::setFingertipVibrotactileFeedbackReferences(
    const yarp::sig::Vector& desiredValue)
{
    if (desiredValue.size() != m_numVibrotactileFeedback)
    {
        yError() << "[GloveControlHelper::setVibroTactileJointsReference] the size of the input "
                    "desired vecotr ["
                 << desiredValue.size() << "] and the number of haptic force feedbacks [ "
                 << m_numVibrotactileFeedback << " ]are not equal.";
        return false;
    }
    for (size_t i = 0; i < m_numVibrotactileFeedback; i++)
    {
        m_desiredVibrotactileValues[i] = desiredValue(i);
    }

    m_pImp->setFingersVibroTactileValues(m_desiredVibrotactileValues);

    return true;
}
bool GloveControlHelper::stopPalmVibrotactileFeedback()
{
    // to implement
    return true;
}

bool GloveControlHelper::stopVibrotactileFeedback()
{
    //    yInfo() << "[GloveControlHelper::turnOffBuzzMotors]";
    //    m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off); // turn off all Buzz Motors.

    for (size_t i = 0; i < m_numVibrotactileFeedback; i++)
    {
        m_desiredVibrotactileValues[i] = 0.0;
    }
    m_pImp->setFingersVibroTactileValues(m_desiredVibrotactileValues);
    return true;
}

bool GloveControlHelper::stopForceFeedback()
{
    for (size_t i = 0; i < m_numForceFeedback; i++)
    {
        m_desiredForceValues[i] = 0.0;
    }

    m_pImp->setFingersForceFeedbackValues(m_desiredForceValues);

    return true;
}

bool GloveControlHelper::stopHapticFeedback()
{

    if (!stopVibrotactileFeedback())
    {
        yError() << "[GloveControlHelper::stopHapticFeedback] Cannot turn off the fingertip "
                    "vibrotactile feedback";
        return false;
    }
    if (!stopForceFeedback())
    {
        yError() << "[GloveControlHelper::stopHapticFeedback] Cannot turn off the fingertip froce "
                    "feedback";
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
    if (!this->stopHapticFeedback())
    {
        yError() << "[GloveControlHelper::close()] Unable to close the glove control helper.";
        return false;
    }
    return true;
}

bool GloveControlHelper::prepareGlove()
{
    std::vector<double> jointAngleList;
    getHandJointAngles(jointAngleList);
    for (size_t i = 0; i < m_humanJointNameList.size(); i++)
    {
        m_jointRangeMin[i] = jointAngleList[i];
        m_jointRangeMax[i] = jointAngleList[i];
    }
    return true;
}

bool GloveControlHelper::setPalmVibrotactileFeedbackReference(const int desiredValue)
{
    // To implement
    return true;
}

bool GloveControlHelper::getHumanHandJointName(const size_t i, std::string& jointName) const
{
    if (i >= this->getNumOfHandJoints())
    {
        yError() << "[GloveControlHelper::getHumanHandJointName] The requested i'th joint name is "
                    "bigger than equal to the number of joints.";
        return false;
    }
    jointName = m_humanJointNameList[i];
    return true;
}

bool GloveControlHelper::getHumanHandJointsNames(std::vector<std::string>& jointNameList) const
{
    if (m_humanJointNameList.size() != this->getNumOfHandJoints())
    {
        yError()
            << "[GloveControlHelper::getHumanHandJointsList] The number of human hand joints and "
               "joints name list size are different.";
        return false;
    }
    jointNameList = m_humanJointNameList;
    return true;
}

bool GloveControlHelper::getHumanHandFingerName(const size_t i, std::string& fingerName) const
{
    if (i >= this->getNumOfFingers())
    {
        yError()
            << "[GloveControlHelper::getHumanHandFingerName] The requested i'th finger name is "
               "bigger than equal to the number of fingers.";
        return false;
    }
    fingerName = m_humanFingerNameList[i];
    return true;
}

bool GloveControlHelper::getHumanHandFingerNames(std::vector<std::string>& fingerNameList) const
{
    if (m_humanFingerNameList.size() != this->getNumOfFingers())
    {
        yError() << "[GloveControlHelper::getHumanFingersList] The number of human hand finger and "
                    "finger name list size are different.";
        return false;
    }
    fingerNameList = m_humanFingerNameList;
    return true;
}

bool GloveControlHelper::findHumanMotionRange()
{
    yInfo() << "GloveControlHelper::findHumanMotionRange";

    std::vector<double> jointAngleList;
    getHandJointAngles(jointAngleList);

    for (size_t i = 0; i < m_humanJointNameList.size(); i++)
    {
        m_jointRangeMin[i] = std::min(m_jointRangeMin[i], jointAngleList[i]);
        m_jointRangeMax[i] = std::max(m_jointRangeMax[i], jointAngleList[i]);
    }

    yarp::sig::Vector desiredValue;
    desiredValue.resize(m_numVibrotactileFeedback, 35);
    setFingertipVibrotactileFeedbackReferences(desiredValue);

    return true;
}

bool GloveControlHelper::getHumanFingerJointsMotionRange(std::vector<double>& jointRangeMin,
                                                         std::vector<double>& jointRangeMax)
{
    yInfo() << "GloveControlHelper::getHumanMotionRange";

    jointRangeMin.resize(m_humanJointNameList.size(), 0.0);
    jointRangeMax.resize(m_humanJointNameList.size(), 0.0);
    for (size_t i = 0; i < m_humanJointNameList.size(); i++)
    {
        jointRangeMax[i] = m_jointRangeMax[i];
        jointRangeMin[i] = m_jointRangeMin[i];
    }
    std::cout << "jointRangeMax" << jointRangeMax << std::endl;
    std::cout << "jointRangeMin" << jointRangeMin << std::endl;
    return true;
}

bool GloveControlHelper::getHandPalmRotation(std::vector<double>& data)
{
    if (!m_pImp->getPalmImuRotationValues(data))
    {
        yError() << "[GloveControlHelper::getGloveIMUData] Unable to get the hand palm IMU data.";
        return false;
    }
    return true;
}
