// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

RetargetingController::~RetargetingController()
{
}
