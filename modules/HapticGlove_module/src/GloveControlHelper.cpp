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
    m_vibroDof = 5;
    m_jointsDof = 25;
    m_isRightHand = rightHand;
    m_desiredVibroValues.resize(m_vibroDof, 0);

    return true;
}

bool GloveControlHelper::setFingersForceReference(const yarp::sig::Vector& desiredValue)
{
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

bool GloveControlHelper::setVibroTactileJointsReference(const yarp::sig::Vector& desiredValue)
{
    if (desiredValue.size() != m_vibroDof)
    {
        yError() << "[GloveControlHelper::setVibroTactileJointsReference] the size of the input "
                    "desired vecotr and the number of buzz motors are not equal.";
        return false;
    }
    for (size_t i = 0; i < m_vibroDof; i++)
    {
        if (desiredValue(i) > 0.0)
            m_desiredVibroValues[i] = (int)std::round(desiredValue(i));
        else
            m_desiredVibroValues[i] = 0;
    }
    m_Glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(
        m_desiredVibroValues)); // vibrate fingers at 80% intensity.
    return true;
}

bool GloveControlHelper::turnOffBuzzMotors()
{

    m_Glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off); // turn off all Buzz Motors.
    return true;
}

void GloveControlHelper::close()
{
}

bool GloveControlHelper::setupGlove()
{
    if (!SGCore::DeviceList::SenseCommRunning()) // Returns true if SenseComm is running.
    {
        yError() << "SenseComm is not running. Please run SenseComm, then try again.";
        return false;
    }
    // GetSenseGlove retrieves the first (connected) Sense Glove it can find. Returns true if one
    // can be found. Additional search parameters can be used.

    if (!SGCore::SG::SenseGlove::GetSenseGlove(m_isRightHand, m_Glove))
    {
        yError() << "No sense gloves connected to the system. Ensure the USB connection is "
                    "secure, then try again.";
        return false;
    }
    yInfo() << "Activating " << m_Glove.ToString();
    return true;
}