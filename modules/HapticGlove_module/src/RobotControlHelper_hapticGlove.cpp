/**
 * @file RobotControlHelper_hapticGlove.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <RobotControlHelper_hapticGlove.hpp>

bool RobotControlHelper::move()
{
    return m_robotControlInterface->setJointReference(m_desiredMotorValue);
}

const std::unique_ptr<HapticGlove::RobotControlInterface>& RobotControlHelper::controlHelper() const
{
    return m_robotControlInterface;
}

std::unique_ptr<HapticGlove::RobotControlInterface>& RobotControlHelper::controlHelper()
{
    return m_robotControlInterface;
}

RobotControlHelper::~RobotControlHelper()
{
}
