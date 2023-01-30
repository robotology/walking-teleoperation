/**
 * @file AdvancedJoypad.hpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2022
 */

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

    void setEyeOpenness(double eyeOpenness);

    bool update();

    void close();

    bool enabled(const yarp::os::ResourceFinder& rf);
};

#endif // ADVANCEDJOYPAD_HPP
