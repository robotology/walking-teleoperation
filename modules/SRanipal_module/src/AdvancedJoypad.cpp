/**
 * @file AdvancedJoypad.hpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2022
 */

#include <AdvancedJoypad.hpp>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>


bool AdvancedJoypad::configure(const yarp::os::ResourceFinder &rf, std::shared_ptr<VRInterface> vrInterface)
{
    if (m_configured)
    {
        yError() << "[AdvancedJoypad::configure] The AdvancedJoypad module is already configured.";
        return false;
    }

    if (!vrInterface)
    {
        yError() << "[AdvancedJoypad::configure] The vrInterface is not valid.";
        return false;
    }

    m_VRInterface = vrInterface;

    if (!enabled(rf))
    {
        return true; //Return early
    }

    yarp::os::Value guisToDisableValue = rf.find("blinkToDisableGUIs");

    if (!guisToDisableValue.isList())
    {
        yError() << "[AdvancedJoypad::configure] The blinkToDisableGUIs is found, but not it is not a list.";
        return false;
    }

    yarp::os::Bottle* guisToDisableList = guisToDisableValue.asList();

    m_guisToDisable.clear();
    for (size_t i = 0; i < guisToDisableList->size(); ++i)
    {
        if (!guisToDisableList->get(i).isInt32())
        {
            yError() << "[AdvancedJoypad::configure] The blinkToDisableGUIs is found, but the element in position" << i << "is not a 32-bit integer.";
            return false;
        }
        m_guisToDisable.push_back(guisToDisableList->get(i).asInt32());
    }

    m_blinkDurationTrigger = rf.check("blinkDurationTrigger", yarp::os::Value(1.0)).asFloat64();
    m_blinkTriggerValue = rf.check("blinkTriggerValue", yarp::os::Value(0.1)).asFloat64();

    m_configured = true;

    m_eyeOpennessValue = 1.0;
    m_isActive = false;
    m_closeInitialTime = -1.0;
    m_eyesAreAlreadyClosed = false;
    return true;
}

void AdvancedJoypad::setEyeOpenness(double eyeOpenness)
{
    m_eyeOpennessValue = eyeOpenness;
}

bool AdvancedJoypad::update()
{
    if (!m_configured)
    {
        yError() << "[AdvancedJoypad::update] The module is not configured yet.";
        return false;
    }

    if (!m_VRInterface->isActive())
    {
        return true;
    }

    if (!m_isActive)
    {
        m_guisEnabled = true;
        for (int gui : m_guisToDisable)
        {
            m_VRInterface->setGUIEnabled(gui, m_guisEnabled);
        }
    }
    m_isActive = true;

    bool eyesAreClosed = m_eyeOpennessValue < m_blinkTriggerValue;

    if (eyesAreClosed && !m_eyesAreAlreadyClosed)
    {
        if (m_closeInitialTime < 0)
        {
            m_closeInitialTime = yarp::os::Time::now();
        }

        if (yarp::os::Time::now() - m_closeInitialTime > m_blinkDurationTrigger)
        {
            yInfo() << "Toggling GUIs!";
            m_eyesAreAlreadyClosed = true;
            m_closeInitialTime = -1.0;

            m_guisEnabled = !m_guisEnabled;
            for (int gui : m_guisToDisable)
            {
                m_VRInterface->setGUIEnabled(gui, m_guisEnabled);
            }
        }
    }
    else if (!eyesAreClosed)
    {
        m_eyesAreAlreadyClosed = false;
        m_closeInitialTime = -1.0;
    }


    return true;
}

void AdvancedJoypad::close()
{
    m_guisEnabled = true;
    for (int gui : m_guisToDisable)
    {
        m_VRInterface->setGUIEnabled(gui, m_guisEnabled);
    }
    m_configured = false;
    m_VRInterface = nullptr;
}

bool AdvancedJoypad::enabled(const yarp::os::ResourceFinder &rf)
{
    return rf.check("blinkToDisableGUIs");
}
