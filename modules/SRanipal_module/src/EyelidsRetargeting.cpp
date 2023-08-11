// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <EyelidsRetargeting.hpp>
#include <yarp/dev/IControlLimits.h>
#include <yarp/os/LogStream.h>
#include <cmath>
#include <algorithm>

bool EyelidsRetargeting::rawRobotEyelidsControl(int eye_closeness_level)
{
    double eye_closeness_leveled = std::max(0.0, std::min(1.0, m_eyelidsPrecision * eye_closeness_level));

    if (eye_closeness_level != m_eyeClosenessLevel)
    {
        yarp::os::Bottle& out = m_rawEyelidsOutputPort.prepare();
        out.clear();
        double eyeOpennesLeveled = 1.0 - eye_closeness_leveled;
        double rawEyelidsValue
                = eyeOpennesLeveled * (m_rawEyelidsOpenValue - m_rawEyelidsCloseValue)
                + m_rawEyelidsCloseValue;
        out.addString("S" + std::to_string(static_cast<int>(std::round(rawEyelidsValue))));
        m_rawEyelidsOutputPort.write();
        m_eyeClosenessLevel = eye_closeness_level;
        yInfo() << "[EyelidsRetargeting::rawRobotEyelidsControl] Setting eye openess (raw mode):" << eyeOpennesLeveled * 100 << "%.";
        yDebug() << "[EyelidsRetargeting::rawRobotEyelidsControl] Sending raw commands to eyelids:" << out.toString();
    }

    return true;
}

bool EyelidsRetargeting::rfeRobotEyelidsControl(int eye_closeness_level)
{
    double eye_closeness_leveled = std::max(0.0, std::min(1.0, m_eyelidsPrecision * eye_closeness_level));


    double eyelidsReference = eye_closeness_leveled * (m_maxEyeLid - m_minEyeLid) + m_minEyeLid; // because min-> open, max->closed
    if (m_useEyelidsPositionControl)
    {
        if (eye_closeness_level != m_eyeClosenessLevel)
        {
            if (m_eyelidsPos)
            {
                m_eyelidsPos->positionMove(0, eyelidsReference);
            }
            m_eyeClosenessLevel = eye_closeness_level;
            yInfo() << "[EyelidsRetargeting::rfeRobotEyelidsControl] Setting eye closeness (position mode):" << eye_closeness_leveled * 100 << "%.";
        }
    }
    else
    {
        if (m_eyelidsVel && m_eyelidsEnc)
        {
            if (eye_closeness_level != m_eyeClosenessLevel)
            {
                m_eyeClosenessLevel = eye_closeness_level;
                yInfo() << "[EyelidsRetargeting::update] Setting eye closeness (velocity mode):" << eye_closeness_leveled * 100 << "%.";
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

    return true;
}

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
    m_eyelidsVelocityGain = rf.check("eyelidsVelocityGain", yarp::os::Value(10.0)).asFloat64();
    m_rawEyelidsCloseValue = rf.check("rawEyelidsCloseValue", yarp::os::Value(35)).asInt32(); //The default value has been found on the greeny
    m_rawEyelidsOpenValue  = rf.check("rawEyelidsOpenValue",  yarp::os::Value(60)).asInt32(); // The default value has been found on the greeny
    m_eyelidsPrecision = rf.check("eyeOpenPrecision", yarp::os::Value(0.1)).asFloat64();

    if (m_eyelidsPrecision <= 0.0)
    {
        yError() << "[EyelidsRetargeting::configure] eyeOpenPrecision has to be strictly positive.";
        return false;
    }

    double defaultMaxVelocity = 100.0;
    if (m_useEyelidsPositionControl)
    {
        defaultMaxVelocity = 75.0;
    }
    m_eyelidsMaxVelocity = rf.check("eyelidsMaxVelocity", yarp::os::Value(defaultMaxVelocity)).asFloat64();


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
                        ok &= m_eyelidsPos->setRefSpeed(0, m_eyelidsMaxVelocity); // max velocity that doesn't give problems
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

void EyelidsRetargeting::setDesiredEyeOpennes(double leftEyeOpennes, double rightEyeOpennes)
{
    m_desiredEyeOpenness = std::min(leftEyeOpennes, rightEyeOpennes);
}

bool EyelidsRetargeting::update()
{
    if (!m_configured)
    {
        yError() << "[EyelidsRetargeting::update] The eyelids retargeting is not configured.";
        return false;
    }

    double desired_eye_closeness = 1.0 - m_desiredEyeOpenness;

    int eye_closeness_level = static_cast<int>(std::round(desired_eye_closeness / m_eyelidsPrecision)); //Using the closeness, in this way when m_eyelidsPrecision is > 1, the robot will close the eyes when the operator has the eyes almost closed

    if (m_useRawEyelids)
    {
        if (!rawRobotEyelidsControl(eye_closeness_level))
        {
            return false;
        }
    }
    else
    {
        if (!rfeRobotEyelidsControl(eye_closeness_level))
        {
            return false;
        }
    }

    return true;
}

void EyelidsRetargeting::close()
{
    if (m_eyelidsMode)
    {
        m_eyelidsMode->setControlMode(0, VOCAB_CM_POSITION);

        if (m_eyelidsPos)
        {
            m_eyelidsPos->positionMove(0, 0.0); //Set the eyes open before closing
        }
        m_eyelidsMode = nullptr;
    }
    m_eyelidsDriver.close();
    m_rawEyelidsOutputPort.close();
    m_eyelidsPos = nullptr;
    m_eyelidsVel = nullptr;
    m_configured = false;
}
