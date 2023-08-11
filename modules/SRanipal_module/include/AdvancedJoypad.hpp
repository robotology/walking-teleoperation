// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ADVANCEDJOYPAD_HPP
#define ADVANCEDJOYPAD_HPP

#include <VRInterface.hpp>

#include <yarp/os/ResourceFinder.h>

#include <vector>
#include <memory>

class AdvancedJoypad
{

    std::shared_ptr<VRInterface> m_VRInterface;
    std::vector<int> m_guisToDisable;
    bool m_guisEnabled{true};
    double m_blinkDurationTrigger;
    double m_blinkTriggerValue;
    double m_eyeOpennessValue{1.0};
    bool m_configured{false};
    bool m_isActive{false};
    bool m_eyesAreAlreadyClosed{false};
    double m_closeInitialTime{-1};

public:

    bool configure(const yarp::os::ResourceFinder& rf, std::shared_ptr<VRInterface> vrInterface);

    void setEyeOpenness(double leftEyeOpennes, double rightEyeOpennes);

    bool update();

    void close();

    bool enabled(const yarp::os::ResourceFinder& rf);
};

#endif // ADVANCEDJOYPAD_HPP
