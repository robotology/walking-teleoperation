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

bool GloveControlHelper::setBuzzMotorsReference(const yarp::sig::Vector& desiredValue)
{
    yInfo() << "[GloveControlHelper::setBuzzMotorsReference]";

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
    m_Glove.SendHaptics(
        SGCore::Haptics::SG_BuzzCmd(m_desiredBuzzValues)); // vibrate fingers at 80% intensity.
    // std::this_thread::sleep_for(std::chrono::milliseconds(10)); // vibrating for for 200ms.
    return true;
}

bool GloveControlHelper::turnOffBuzzMotors()
{
    yInfo() << "[GloveControlHelper::turnOffBuzzMotors]";
    m_Glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off); // turn off all Buzz Motors.
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

    if (!SGCore::SG::SenseGlove::GetSenseGlove(m_isRightHand, m_Glove))
    {
        yError() << "No sense gloves connected to the system. Ensure the USB connection is "
                    "secure, then try again.";
        return false;
    }

    yInfo() << "Activating " << m_Glove.ToString();
    return true;
}