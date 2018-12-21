/**
 * @file RetargetingController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <RetargetingController.hpp>

bool RetargetingController::move()
{
    return m_controlHelper->setJointReference(m_desiredJointValue);
}

const std::unique_ptr<RobotControlHelper>& RetargetingController::controlHelper() const
{
    return m_controlHelper;
}

std::unique_ptr<RobotControlHelper>& RetargetingController::controlHelper()
{
    return m_controlHelper;
}
