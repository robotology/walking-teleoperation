/**
 * @file GazeRetargeting.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <GazeRetargeting.hpp>
#include <yarp/os/LogStream.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/os/Vocab.h>
#include <iDynTree/Core/Utils.h>
#include <yarp/os/Time.h>
#include <algorithm>
#include <cmath>


void GazeRetargeting::setRobotEyeControlMode(int controlMode)
{
    if (m_eyesMode)
    {
        for (auto& mode : m_eyeControlModes)
        {
            mode = controlMode;
        }
        m_eyesMode->setControlModes(m_eyeAxis.size(), m_eyeAxis.data(), m_eyeControlModes.data());
    }
}

bool GazeRetargeting::homeRobotEyes()
{
    if (!m_eyesPos)
    {
        return false;
    }

    if (!updateRobotEyeEncoders())
    {
        return false;
    }

    yInfo() << "[GazeRetargeting::homeRobotEyes] Homing robot eyes..";

    double maxError = std::max({m_eyeTiltInRad, m_eyeVersInRad, m_eyeVergInRad});
    double expectedTime = maxError / iDynTree::deg2rad(m_maxEyeSpeedInDegS);


    m_eyesPos->setRefSpeeds(m_eyeAxis.size(), m_eyeAxis.data(), m_eyemaxPositionMoveSpeeds.data());

    if (!m_eyesPos->positionMove(m_eyeAxis.size(), m_eyeAxis.data(), m_eyePositionReferences.data()))
    {
        return false;
    }

    yarp::os::Time::delay(3.0 * expectedTime); //Just give some time for it to go to home

    yInfo() << "[GazeRetargeting::homeRobotEyes] Robot eyes homed!";

    return true;
}

bool GazeRetargeting::updateRobotEyeEncoders()
{
    if (!m_eyesEnc || !m_eyesEnc->getEncoders(m_encodersInDeg.data()))
    {
        return false;
    }

    m_eyeVersInRad = m_useVersion ? iDynTree::deg2rad(m_encodersInDeg[m_eyeVersIndex]) : 0.0;
    m_eyeVergInRad = m_useVergence ? iDynTree::deg2rad(m_encodersInDeg[m_eyeVergIndex]) : 0.0;
    m_eyeTiltInRad = m_useTilt ? iDynTree::deg2rad(m_encodersInDeg[m_eyeTiltIndex]) : 0.0;

    return true;
}

bool GazeRetargeting::setDesiredRobotEyeVelocities(double vergenceSpeedInDegS, double versionSpeedInDegS, double tiltSpeedInDegS)
{
    if (!m_eyesVel)
    {
        return false;
    }

    *m_versionVelocityptr = versionSpeedInDegS;
    *m_vergenceVelocityptr = vergenceSpeedInDegS;
    *m_tiltVelocityptr = tiltSpeedInDegS;

    return m_eyesVel->velocityMove(m_eyeAxis.size(), m_eyeAxis.data(), m_eyesVelocityReferences.data());
}

double GazeRetargeting::saturateRobotEyeVelocity(double inputVelocity, double inputPosition, double maxVelocity, double jointLowerBound, double jointUpperBound)
{
    //See https://github.com/ami-iit/element_qp-reactive-control/issues/51
    double velocityLowerLimit = std::tanh(m_tanhGain * (inputPosition - jointLowerBound)) * (-maxVelocity);
    double velocityUpperLimit = std::tanh(m_tanhGain * (jointUpperBound - inputPosition)) * maxVelocity;

    return std::max(velocityLowerLimit, std::min(inputVelocity, velocityUpperLimit));
}

GazeRetargeting::~GazeRetargeting()
{
    close();
}

bool GazeRetargeting::configure(const yarp::os::ResourceFinder &rf, std::shared_ptr<VRInterface> vrInterface)
{
    if (m_configured)
    {
        yError() << "[GazeRetargeting::configure] The gaze retargeting is already configured.";
        return false;
    }

    if (!vrInterface)
    {
        yError() << "[GazeRetargeting::configure] The input vr interface is not valid.";
        return false;
    }
    m_VRInterface = vrInterface;

    std::string name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    std::string robot = rf.check("robot", yarp::os::Value("icub"), "The name of the robot to connect to.").asString();

    std::string eyes_version_name =  rf.check("eyesVersionName", yarp::os::Value("eyes_vers")).asString();
    m_useVersion = eyes_version_name.find("none") == std::string::npos;
    std::string eyes_vergence_name = rf.check("eyesVergenceName", yarp::os::Value("eyes_verg")).asString();
    m_useVergence = eyes_vergence_name.find("none") == std::string::npos;
    std::string eyes_tilt_name = rf.check("eyesTiltName", yarp::os::Value("eyes_tilt")).asString();
    m_useTilt = eyes_tilt_name.find("none") == std::string::npos;

    if (!m_useVersion && !m_useVergence && !m_useTilt)
    {
        yError() << "[GazeRetargeting::configure] No robot joint enabled. Use --noGaze to disable the gaze retargeting.";
        return false;
    }

    m_maxEyeSpeedInDegS = rf.check("eyeMaxVelocity", yarp::os::Value(20.0)).asFloat64();
    double userMaxVergInDeg = rf.check("eyeMaxVergence", yarp::os::Value(10.0)).asFloat64();
    double userMaxVersInDeg = rf.check("eyeMaxVersion", yarp::os::Value(25.0)).asFloat64();
    double userMaxTiltInDeg = rf.check("eyeMaxTilt", yarp::os::Value(30.0)).asFloat64();
    m_tanhGain = rf.check("eyeKinematicSaturationGain", yarp::os::Value(10.0)).asFloat64();
    std::string headControlBoard = rf.check("headControlBoardName", yarp::os::Value("head")).asString();

    yarp::os::Property rcb_head_conf{
        {"device", yarp::os::Value("remote_controlboard")},
        {"local", yarp::os::Value("/" + name + "/head/remoteControlBoard")},
        {"remote", yarp::os::Value("/" + robot + "/" + headControlBoard)},
        {"part", yarp::os::Value(headControlBoard)}};

    if (!m_eyesDriver.open(rcb_head_conf))
    {
        yError() << "[GazeRetargeting::configure] Failed to open the head control board. Use noGaze to avoid connecting to it.";
        return false;
    }

    yarp::dev::IAxisInfo* axisInfo{nullptr};
    yarp::dev::IControlLimits* controlLimits{nullptr};

    if (!m_eyesDriver.view(axisInfo) || !axisInfo)
    {
        yError() << "[GazeRetargeting::configure] Failed to view the IAxisInfo interface. Use noGaze to avoid connecting to it.";
        return false;
    }

    if (!m_eyesDriver.view(controlLimits) || !controlLimits)
    {
        yError() << "[GazeRetargeting::configure] Failed to view the IControlLimits interface. Use noGaze to avoid connecting to it.";
        return false;
    }

    if (!m_eyesDriver.view(m_eyesPos) || !m_eyesPos)
    {
        yError() << "[GazeRetargeting::configure] Failed to view the IPositionControl interface. Use noGaze to avoid connecting to it.";
        return false;
    }

    if (!m_eyesDriver.view(m_eyesVel) || !m_eyesVel)
    {
        yError() << "[GazeRetargeting::configure] Failed to view the IVelocityControl interface. Use noGaze to avoid connecting to it.";
        return false;
    }

    if (!m_eyesDriver.view(m_eyesEnc) || !m_eyesEnc)
    {
        yError() << "[GazeRetargeting::configure] Failed to view the IEncoders interface. Use noGaze to avoid connecting to it.";
        return false;
    }

    if (!m_eyesDriver.view(m_eyesMode) || !m_eyesMode)
    {
        yError() << "[GazeRetargeting::configure] Failed to view the IControlMode interface. Use noGaze to avoid connecting to it.";
        return false;
    }

    int nAxes = 0;
    if (!m_eyesVel->getAxes(&nAxes))
    {
        yError() << "[GazeRetargeting::configure] Failed to get the number of axes. Use noGaze to avoid connecting to it.";
        return false;
    }

    m_encodersInDeg.resize(nAxes);

    m_eyeTiltIndex = -1;
    m_eyeVergIndex = -1;
    m_eyeVersIndex = -1;

    for (size_t i = 0; i < nAxes; ++i)
    {
        std::string axisName;
        if (!axisInfo->getAxisName(i, axisName))
        {
            yError() << "[GazeRetargeting::configure] Failed to get the axis name of the neck joint with index" << i <<"." ;
            return false;
        }

        if (m_useVersion && (axisName.find(eyes_version_name) != std::string::npos))
        {
            m_eyeVersIndex = i;
        }
        else if (m_useVergence && (axisName.find(eyes_vergence_name) != std::string::npos))
        {
            m_eyeVergIndex = i;
        }
        else if (m_useTilt && (axisName.find(eyes_tilt_name) != std::string::npos))
        {
            m_eyeTiltIndex = i;
        }
    }

    if (m_useVersion && m_eyeVersIndex < 0)
    {
        yError() << "[GazeRetargeting::configure] Failed to find the version joint with the name" << eyes_version_name << "among the neck joints." ;
        return false;
    }
    if (m_useVergence && m_eyeVergIndex < 0)
    {
        yError() << "[GazeRetargeting::configure] Failed to find the vergence joint with the name" << eyes_vergence_name << "among the neck joints." ;
        return false;
    }
    if (m_useTilt && m_eyeTiltIndex < 0)
    {
        yError() << "[GazeRetargeting::configure] Failed to find the tilt joint with the name" << eyes_tilt_name << "among the neck joints." ;
        return false;
    }

    m_eyeAxis.clear();
    m_versionVelocityptr = &m_dummy;
    m_vergenceVelocityptr = &m_dummy;
    m_tiltVelocityptr = &m_dummy;
    if (m_useVersion)
    {
        double robotMinVersInDeg, robotMaxVersInDeg;
        if (!controlLimits->getLimits(m_eyeVersIndex, &robotMinVersInDeg, &robotMaxVersInDeg))
        {
            yError() << "[GazeRetargeting::configure] Failed to get the control limits of the eyes "
                        "version.";
            return false;
        }
        m_eyesVel->setRefAcceleration(m_eyeVersIndex, std::numeric_limits<double>::max());
        m_maxVersInDeg = std::min(
            {userMaxVersInDeg, std::abs(robotMinVersInDeg), std::abs(robotMaxVersInDeg)});
        m_eyeAxis.push_back(m_eyeVersIndex);
        m_eyeControlModes.push_back(VOCAB_CM_POSITION);
        m_eyemaxPositionMoveSpeeds.push_back(m_maxEyeSpeedInDegS);
        m_eyePositionReferences.push_back(0.0);
        m_eyesVelocityReferences.push_back(0.0);
        m_versionVelocityptr = &m_eyesVelocityReferences.back();
    } else
    {
        yInfo() << "[GazeRetargeting::configure] Not using version joint.";
    }

    if (m_useVergence)
    {
        double robotMinVergInDeg, robotMaxVergInDeg;
        if (!controlLimits->getLimits(m_eyeVergIndex, &robotMinVergInDeg, &robotMaxVergInDeg))
        {
            yError() << "[GazeRetargeting::configure] Failed to get the control limits of the eyes "
                        "vergence.";
            return false;
        }
        m_eyesVel->setRefAcceleration(m_eyeVergIndex, std::numeric_limits<double>::max());
        m_maxVergInDeg = std::min(userMaxVergInDeg,
                                  robotMaxVergInDeg); // The min vergence is supposed to be 0.0
        m_eyeAxis.push_back(m_eyeVergIndex);
        m_eyeControlModes.push_back(VOCAB_CM_POSITION);
        m_eyemaxPositionMoveSpeeds.push_back(m_maxEyeSpeedInDegS);
        m_eyePositionReferences.push_back(0.0);
        m_eyesVelocityReferences.push_back(0.0);
        m_vergenceVelocityptr = &m_eyesVelocityReferences.back();
    } else
    {
        yInfo() << "[GazeRetargeting::configure] Not using vergence joint.";
    }

    if (m_useTilt)
    {
        double robotMinTiltInDeg, robotMaxTiltInDeg;
        if (!controlLimits->getLimits(m_eyeTiltIndex, &robotMinTiltInDeg, &robotMaxTiltInDeg))
        {
            yError() << "[GazeRetargeting::configure] Failed to get the control limits of the eyes "
                        "tilt.";
            return false;
        }
        m_eyesVel->setRefAcceleration(m_eyeTiltIndex, std::numeric_limits<double>::max());
        m_maxTiltInDeg = std::min(
            {userMaxTiltInDeg, std::abs(robotMinTiltInDeg), std::abs(robotMaxTiltInDeg)});
        m_eyeAxis.push_back(m_eyeTiltIndex);
        m_eyeControlModes.push_back(VOCAB_CM_POSITION);
        m_eyemaxPositionMoveSpeeds.push_back(m_maxEyeSpeedInDegS);
        m_eyePositionReferences.push_back(0.0);
        m_eyesVelocityReferences.push_back(0.0);
        m_tiltVelocityptr = &m_eyesVelocityReferences.back();
    } else
    {
        yInfo() << "[GazeRetargeting::configure] Not using tilt joint.";
    }

    setRobotEyeControlMode(VOCAB_CM_POSITION);

    homeRobotEyes();

    setRobotEyeControlMode(VOCAB_CM_VELOCITY);

    m_configured = true;

    return true;
}

void GazeRetargeting::setOperatorEyeGazeAxes(const iDynTree::Axis &leftGaze, const iDynTree::Axis &rightGaze)
{
    m_leftGazeOperator.setDirection(leftGaze.getDirection());
    m_leftGazeOperator.setOrigin(leftGaze.getOrigin());
    m_rightGazeOperator.setDirection(rightGaze.getDirection());
    m_rightGazeOperator.setOrigin(rightGaze.getOrigin());
    m_gazeSet = true;
}

bool GazeRetargeting::update()
{
    if (!m_configured)
    {
        yError() << "[GazeRetargeting::update] The gaze retargeting module is not initialized.";
        return false;
    }

    if (!m_VRInterface->isActive())
    {
        return true; //do nothing
    }

    //Get the current eye encoder values
    if (!updateRobotEyeEncoders())
    {
        yError() << "[GazeRetargeting::update] Failed to get eye encoders.";
        return false;
    }

    double vergenceSpeedInRadS = 0.0, versionSpeedInRadS = 0.0, tiltSpeedInRadS = 0.0;

    //Compute the desired eye speed according to the user gaze
    if (m_gazeSet) //The desired gaze has been set at least once.
    {
        if (!m_VRInterface->computeDesiredRobotEyeVelocities(m_leftGazeOperator, m_rightGazeOperator, vergenceSpeedInRadS, versionSpeedInRadS, tiltSpeedInRadS))
        {
            yError() << "[GazeRetargeting::update] Failed to compute the desired eye velocity.";
            return false;
        }
    }

    double vergenceSpeedInDegS = iDynTree::rad2deg(vergenceSpeedInRadS);
    double versionSpeedInDegS = iDynTree::rad2deg(versionSpeedInRadS);
    double tiltSpeedInDegS = iDynTree::rad2deg(tiltSpeedInRadS);

    //We saturate the desired eye velocities according to the limits too
    vergenceSpeedInDegS = m_useVergence ? saturateRobotEyeVelocity(vergenceSpeedInDegS, m_encodersInDeg[m_eyeVergIndex], m_maxEyeSpeedInDegS, 0.0, m_maxVergInDeg) : 0.0;
    versionSpeedInDegS = m_useVersion ? saturateRobotEyeVelocity(versionSpeedInDegS, m_encodersInDeg[m_eyeVersIndex], m_maxEyeSpeedInDegS, -m_maxVersInDeg, m_maxVersInDeg) : 0.0;
    tiltSpeedInDegS = m_useTilt ? saturateRobotEyeVelocity(tiltSpeedInDegS, m_encodersInDeg[m_eyeTiltIndex], m_maxEyeSpeedInDegS, -m_maxTiltInDeg, m_maxTiltInDeg) : 0.0;

    //Set the desired velocitites to the robot
    if (!setDesiredRobotEyeVelocities(vergenceSpeedInDegS, versionSpeedInDegS, tiltSpeedInDegS))
    {
        yError() << "[GazeRetargeting::update] Failed to set the desired eye velocity.";
        return false;
    }

    //Update the orientation of the images in the VR view.
    m_VRInterface->setVRImagesPose(m_eyeVergInRad, m_eyeVersInRad, m_eyeTiltInRad);

    return true;
}

void GazeRetargeting::close()
{
    setRobotEyeControlMode(VOCAB_CM_POSITION);
    homeRobotEyes();
    m_eyesDriver.close();
    m_eyesVel = nullptr;
    m_eyesPos = nullptr;
    m_eyesMode = nullptr;
    m_eyesEnc = nullptr;
    m_configured = false;
    m_VRInterface = nullptr;
}
