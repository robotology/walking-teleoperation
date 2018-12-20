/**
 * @file RetargetingHelper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <RetargetingHelper.hpp>

bool RetargetingHelper::move()
{
    return m_controlHelper->setDirectPositionReferences(m_desiredJointPosition);
}

const std::unique_ptr<RobotControlHelper>& RetargetingHelper::controlHelper() const
{
    return m_controlHelper;
}

std::unique_ptr<RobotControlHelper>& RetargetingHelper::controlHelper()
{
    return m_controlHelper;
}
