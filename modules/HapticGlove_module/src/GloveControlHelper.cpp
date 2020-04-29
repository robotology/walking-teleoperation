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

using namespace HapticGlove;

bool GloveControlHelper::configure(const yarp::os::Searchable& config,
                                   const std::string& name,
                                   const bool& rightHand)
{

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    m_isReady = false;
    m_forceFbDof = 5;
    m_buzzDof = 5;
    m_jointsDof = 25;
    m_isRightHand = rightHand;
    m_desiredBuzzValues.resize(m_buzzDof, 0);
    m_desiredForceValues.resize(m_forceFbDof, 0);

    return true;
}

bool GloveControlHelper::setFingersForceReference(const yarp::sig::Vector& desiredValue)
{
    if (desiredValue.size() != m_forceFbDof)
    {
        yError() << "[GloveControlHelper::setFingersForceReference] the size of the input "
                    "desired vecotr and the number of haptic force feedbacks are not equal.";
        return false;
    }

        for (size_t i = 0; i < m_forceFbDof; i++)
    {
        if (desiredValue(i) > 0.0)
            m_desiredForceValues[i] = (int)std::round(desiredValue(i));
        else
            m_desiredForceValues[i] = 0;
        std::cout << m_desiredForceValues[i] << " ";
    }
    std::cout << std::endl;


    if (!m_glove.SendHaptics(SGCore::Haptics::SG_FFBCmd(m_desiredForceValues)))
    {
        yError() << "[GloveControlHelper::setFingersForceReference] unable the send the force feedback command";
        return false;
    } 
 
    return true;
}

bool GloveControlHelper::getFingersForceMeasured(yarp::sig::Vector& measuredValue)
{
    return true;
}

bool GloveControlHelper::getFingersJointsMeasured(yarp::sig::Vector& measuredValue)
{
    return true;
}

bool GloveControlHelper::setBuzzMotorsReference(const yarp::sig::Vector& desiredValue)
{
    if (desiredValue.size() != m_buzzDof)
    {
        yError() << "[GloveControlHelper::setVibroTactileJointsReference] the size of the input "
                    "desired vecotr and the number of buzz motors are not equal.";
        return false;
    }
    for (size_t i = 0; i < m_buzzDof; i++)
    {
        if (desiredValue(i) > 0.0)
            m_desiredBuzzValues[i] = (int)std::round(desiredValue(i));
        else
            m_desiredBuzzValues[i] = 0;
        std::cout << m_desiredBuzzValues[i] << " ";
    }
    std::cout << std::endl;
    // vibrate fingers at percetage intensity, between 0-100, integer numbers
   if( !m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(m_desiredBuzzValues)))
    {
       yError() << "[GloveControlHelper::setBuzzMotorsReference] unable the send the Buzz command";
       return false;
   } 
    
    return true;
}

bool GloveControlHelper::turnOffBuzzMotors()
{
    yInfo() << "[GloveControlHelper::turnOffBuzzMotors]";
    m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off); // turn off all Buzz Motors.
    return true;
}

int GloveControlHelper::getNoOfBuzzMotors()
{
    return m_buzzDof;
}

void GloveControlHelper::close()
{
}

bool GloveControlHelper::setupGlove()
{
    yInfo() << "GloveControlHelper::setupGlove()";

    if (!SGCore::DeviceList::SenseCommRunning()) // Returns true if SenseComm is running.
    {
        yError() << "SenseComm is not running. Please run SenseComm, then try again.";
        return false;
    }
    // GetSenseGlove retrieves the first (connected) Sense Glove it can find. Returns true if one
    // can be found. Additional search parameters can be used.

    if (!SGCore::SG::SenseGlove::GetSenseGlove(m_isRightHand, m_glove))
    {
        yError() << "No sense gloves connected to the system. Ensure the USB connection is "
                    "secure, then try again.";
        return false;
    }

    yInfo() << "Activating " << m_glove.ToString();
    return true;
}

bool GloveControlHelper::stopFeedback()
{
    return m_glove.StopFeedback();
}


bool GloveControlHelper::setPalmFeedback(const int desiredValue)
{

    return m_glove.SendHaptics(SGCore::Haptics::Impact_Thump_100);
}
    ///// <summary> set the level(s) of force and vibrotactile feedback, with an optional thumper
///command
///// </summary>
// bool sendhaptics(haptics::sg_ffbcmd ffbcmd,
//                 haptics::sg_buzzcmd buzzcmd,
//                 haptics::sg_thumpercmd thumpercmd = haptics::sg_thumpercmd::none);
//
///// <summary> send a force-feedback command to the sense glove. </summary>
// bool sendhaptics(haptics::sg_ffbcmd ffbcmd);
//
///// <summary> send a vibration command to the sense glove. </summary>
// bool sendhaptics(haptics::sg_buzzcmd buzzcmd);
//
///// <summary> send a thumper command. </summary>
// bool sendhaptics(haptics::sg_thumpercmd thumpercmd);
//
///// <summary> stop all haptic feedback on this device. </summary>
// bool stopfeedback();
