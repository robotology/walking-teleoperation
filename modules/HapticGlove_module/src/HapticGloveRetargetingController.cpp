/**
 * @file HapticGloveRetargetingController.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <HapticGloveRetargetingController.hpp>

bool RetargetingController::move()
{
    return m_controlHelper->setJointReference(m_desiredMotorValue);
}

const std::unique_ptr<HapticGlove::RobotControlHelper>& RetargetingController::controlHelper() const
{
    return m_controlHelper;
}

std::unique_ptr<HapticGlove::RobotControlHelper>& RetargetingController::controlHelper()
{
    return m_controlHelper;
}

RetargetingController::~RetargetingController()
{
}
