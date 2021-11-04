/**
     * @file EyelidsRetargeting.cpp
     * @authors Stefano Dafarra <stefano.dafarra@iit.it>
     * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
     *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
     * @date 2021
     */


#include <EyelidsRetargeting.hpp>
#include <yarp/dev/IControlLimits.h>
#include <yarp/os/LogStream.h>
#include <cmath>
#include <algorithm>

EyelidsRetargeting::~EyelidsRetargeting()
{
    close();
}

bool EyelidsRetargeting::configure(yarp::os::ResourceFinder &rf)
{
    if (m_configured)
    {
        yError() << "[EyelidsRetargeting::configure] The eyelids retargeting is already configured.";
        return false;
    }

    std::string name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    std::string robot = rf.check("robot", yarp::os::Value("icub"), "The name of the robot to connect to.").asString();
    m_useRawEyelids = rf.check("useRawEyelids")
            && (rf.find("useRawEyelids").isNull() || rf.find("useRawEyelids").asBool());
    m_useEyelidsPositionControl = rf.check("useEyelidsPositionControl")
            && (rf.find("useEyelidsPositionControl").isNull() || rf.find("useEyelidsPositionControl").asBool());
    m_eyelidsMaxVelocity = rf.check("eyelidsMaxVelocity", yarp::os::Value(100.0)).asFloat64();
    m_eyelidsVelocityGain = rf.check("eyelidsVelocityGain", yarp::os::Value(100.0)).asFloat64();
    m_rawEyelidsCloseValue = rf.check("rawEyelidsCloseValue", yarp::os::Value(35)).asInt32(); //The default value has been found on the greeny
    m_rawEyelidsOpenValue  = rf.check("rawEyelidsOpenValue",  yarp::os::Value(60)).asInt32(); // The default value has been found on the greeny
    m_eyeOpenPrecision = rf.check("eyeOpenPrecision", yarp::os::Value(0.1)).asFloat64();
    std::string rawEyelidsPortName = rf.check("rawEyelidsPortName", yarp::os::Value("/face/raw:o")).asString();
    if (m_useRawEyelids)
    {
        if (!m_rawEyelidsOutputPort.open("/" + name + rawEyelidsPortName))
        {
            yError() << "[EyelidsRetargeting::configure] Failed to open /" + name
                        + rawEyelidsPortName
                        + " port.";
            return false;
        }
    }
    else
    {
        yarp::os::Property rcb_face_conf{
            {"device", yarp::os::Value("remote_controlboard")},
            {"local", yarp::os::Value("/" + name + "/face/remoteControlBoard")},
            {"remote", yarp::os::Value("/" + robot + "/face")},
            {"part", yarp::os::Value("face")}};

        if (m_eyelidsDriver.open(rcb_face_conf))
        {
            yarp::dev::IControlLimits* iCtrlLim{nullptr};
            bool ok = m_eyelidsDriver.view(m_eyelidsMode);
            ok &= m_eyelidsDriver.view(m_eyelidsPos);
            ok &= m_eyelidsDriver.view(m_eyelidsVel);
            ok &= m_eyelidsDriver.view(m_eyelidsEnc);
            ok &= m_eyelidsDriver.view(iCtrlLim);

            if (!ok)
            {
                yError() << "[EyelidsRetargeting::configure] Failed to configure the eyelids remote_controlboard.";
                return false;
            }

            if (m_eyelidsMode)
            {
                if (m_useEyelidsPositionControl)
                {
                    ok &= m_eyelidsMode->setControlMode(0, VOCAB_CM_POSITION);
                    if (m_eyelidsPos)
                    {
                        ok &= m_eyelidsPos->setRefSpeed(0, 75.0); // max velocity that doesn't give problems
                        ok &= m_eyelidsPos->setRefAcceleration(0, std::numeric_limits<double>::max());
                    }
                    else
                    {
                        ok = false;
                    }
                }
                else
                {
                    ok &= m_eyelidsMode->setControlMode(0, VOCAB_CM_VELOCITY);
                    if (m_eyelidsVel)
                    {
                        ok &= m_eyelidsVel->setRefAcceleration(0, std::numeric_limits<double>::max());
                    }
                    else
                    {
                        ok = false;
                    }
                }

                if (!ok)
                {
                    yError() << "[EyelidsRetargeting::configure] Failed to configure the eyelids control.";
                    return false;
                }
            }

            if (iCtrlLim)
            {
                ok &= iCtrlLim->getLimits(0, &m_minEyeLid, &m_maxEyeLid);
                m_maxEyeLid = 0.9 * m_maxEyeLid;
            }
            else
            {
                ok = false;
            }

            if (!ok)
            {
                yError() << "[EyelidsRetargeting::configure] Failed to get the eyelids limits.";
                return false;
            }

        }
        else
        {
            yError() << "[EyelidsRetargeting::configure] Failed to connect to the face control board. Set noEyelids to "
                        "true to avoid connecting to it.";
            return false;
        }
    }

    m_configured = true;
    return true;
}

bool EyelidsRetargeting::usingEylidsVelocityControl()
{
    return m_configured && !(m_useRawEyelids || m_useEyelidsPositionControl);
}

void EyelidsRetargeting::setDesiredEyeOpennes(double eyeOpennes)
{
    m_desiredEyeOpennes = eyeOpennes;
}

bool EyelidsRetargeting::update()
{
    if (!m_configured)
    {
        yError() << "[EyelidsRetargeting::update] The eyelids retargeting is not configured.";
        return false;
    }

    int eye_open_level = static_cast<int>(std::round(m_desiredEyeOpennes / m_eyeOpenPrecision));
    double eye_openess_leveled = m_eyeOpenPrecision * eye_open_level;

    if (m_useRawEyelids)
    {
        if (eye_open_level != m_eyeOpenLevel)
        {
            yarp::os::Bottle& out = m_rawEyelidsOutputPort.prepare();
            out.clear();
            double rawEyelidsValue
                    = eye_openess_leveled * (static_cast<double>(m_rawEyelidsOpenValue) - m_rawEyelidsCloseValue)
                    + m_rawEyelidsCloseValue;
            out.addString("S" + std::to_string(static_cast<int>(std::round(rawEyelidsValue))));
            m_rawEyelidsOutputPort.write();
            m_eyeOpenLevel = eye_open_level;
            yInfo() << "[EyelidsRetargeting::update] Setting eye openess (raw):" << eye_openess_leveled;
            yDebug() << "[EyelidsRetargeting::update] Sending raw commands to eyelids:" << out.toString();
        }
    }
    else
    {
        double eyelidsReference = (1.0 - eye_openess_leveled) * (m_maxEyeLid - m_minEyeLid) + m_minEyeLid; // because min-> open, max->closed
        if (m_useEyelidsPositionControl)
        {
            if (eye_open_level != m_eyeOpenLevel)
            {
                if (m_eyelidsPos)
                {
                    m_eyelidsPos->positionMove(0, eyelidsReference); // because min-> open, max->closed
                }
                m_eyeOpenLevel = eye_open_level;
                yInfo() << "[EyelidsRetargeting::update] Setting eye openess (position):" << eye_openess_leveled;
            }
        }
        else
        {
            if (m_eyelidsVel && m_eyelidsEnc)
            {
                if (eye_open_level != m_eyeOpenLevel)
                {
                    m_eyeOpenLevel = eye_open_level;
                    yInfo() << "[EyelidsRetargeting::update] Setting eye openess (velocity):" << eye_openess_leveled;
                }

                double currentEyelidPos = 0;
                if (m_eyelidsEnc->getEncoder(0, &currentEyelidPos))
                {
                    double eyelidsVelocityReference = m_eyelidsVelocityGain * (eyelidsReference - currentEyelidPos);
                    double eyelidsVelClamped = std::max(-m_eyelidsMaxVelocity, std::min(eyelidsVelocityReference, m_eyelidsMaxVelocity)); //Clamp the velocity reference in [-max, +max];
                    m_eyelidsVel->velocityMove(0, eyelidsVelClamped);
                }
                else
                {
                    yWarning() << "[EyelidsRetargeting::update] Failed to get eyelids encoder value. Skipping control.";
                    return false;
                }
            }
        }
    }

    return true;
}

void EyelidsRetargeting::close()
{
    if (m_eyelidsMode)
    {
        m_eyelidsMode->setControlMode(0, VOCAB_CM_POSITION);
        m_eyelidsMode = nullptr;
    }
    m_eyelidsDriver.close();
    m_rawEyelidsOutputPort.close();
    m_eyelidsPos = nullptr;
    m_eyelidsVel = nullptr;
    m_configured = false;
}
