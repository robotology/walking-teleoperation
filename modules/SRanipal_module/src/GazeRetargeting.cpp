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
#include <iDynTree/Core/Utils.h>
#include <yarp/os/Time.h>
#include <cmath>
#include <algorithm>


void GazeRetargeting::setEyeControlMode(int controlMode)
{
    if (m_eyesMode)
    {
        int eyeAxis[] = {m_eyeTiltIndex, m_eyeVersIndex, m_eyeVergIndex};
        int controlModes[] = {controlMode, controlMode, controlMode};
        m_eyesMode->setControlModes(3, eyeAxis, controlModes);
    }
}

bool GazeRetargeting::homeEyes()
{
    if (!m_eyesPos)
    {
        return false;
    }

    if (!updateEyeEncoders())
    {
        return false;
    }

    int eyeAxis[] = {m_eyeTiltIndex, m_eyeVersIndex, m_eyeVergIndex};
    double speeds[] = {m_maxEyeSpeed, m_maxEyeSpeed, m_maxEyeSpeed};
    double references[] = {0.0, 0.0, 0.0};

    double maxError = std::max({m_eyeTiltInRad, m_eyeVersInRad, m_eyeVergInRad});
    double expectedTime = maxError / iDynTree::deg2rad(m_maxEyeSpeed);


    m_eyesPos->setRefSpeeds(3, eyeAxis, speeds);

    if (!m_eyesPos->positionMove(3, eyeAxis, references))
    {
        return false;
    }

    yarp::os::Time::delay(3.0 * expectedTime); //Just give some time for it to go to home

    return true;
}

bool GazeRetargeting::updateEyeEncoders()
{
    if (!m_eyesEnc || !m_eyesEnc->getEncoders(m_encodersInDeg.data()))
    {
        return false;
    }

    m_eyeVersInRad = iDynTree::deg2rad(m_encodersInDeg[m_eyeVersIndex]);
    m_eyeVergInRad = iDynTree::deg2rad(m_encodersInDeg[m_eyeVergIndex]);
    m_eyeTiltInRad = iDynTree::deg2rad(m_encodersInDeg[m_eyeTiltIndex]);

    return true;
}

bool GazeRetargeting::setDesiredEyeVelocities(double vergenceSpeed, double versionSpeed, double tiltSpeed)
{
    if (!m_eyesVel)
    {
        return false;
    }

    double velRefs[] = {vergenceSpeed, versionSpeed, tiltSpeed};
    int eyeAxis[] = {m_eyeTiltIndex, m_eyeVersIndex, m_eyeVergIndex};

    return m_eyesVel->velocityMove(3, eyeAxis, velRefs);
}

GazeRetargeting::~GazeRetargeting()
{
    close();
}

bool GazeRetargeting::configure(yarp::os::ResourceFinder &rf)
{
    if (m_configured)
    {
        yError() << "[GazeRetargeting::configure] The gaze retargeting is already configured.";
        return false;
    }

    std::string name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    std::string robot = rf.check("robot", yarp::os::Value("icub"), "The name of the robot to connect to.").asString();

    std::string eyes_version_name =  rf.check("eyes_version_name", yarp::os::Value("eyes_vers")).asString();
    std::string eyes_vergence_name = rf.check("eyes_vergence_name", yarp::os::Value("eyes_verg")).asString();
    std::string eyes_tilt_name = rf.check("eyes_tilt_name", yarp::os::Value("eyes_tilt")).asString();

    m_maxEyeSpeed = rf.check("eyeMaxVelocity", yarp::os::Value(50.0)).asFloat64();

    yarp::os::Property rcb_head_conf{
        {"device", yarp::os::Value("remote_controlboard")},
        {"local", yarp::os::Value("/" + name + "/head/remoteControlBoard")},
        {"remote", yarp::os::Value("/" + robot + "/head")},
        {"part", yarp::os::Value("head")}};

    if (!m_eyesDriver.open(rcb_head_conf))
    {
        yError() << "[GazeRetargeting::configure] Failed to open the head control board. Use noGaze to avoid connecting to it.";
        return false;
    }

    yarp::dev::IAxisInfo* axisInfo{nullptr};

    if (!m_eyesDriver.view(axisInfo) || !axisInfo)
    {
        yError() << "[GazeRetargeting::configure] Failed to view the IAxisInfo interface. Use noGaze to avoid connecting to it.";
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

    m_eyesVel->setRefAcceleration(0, std::numeric_limits<double>::max());

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

        if (axisName.find(eyes_version_name) != std::string::npos)
        {
            m_eyeVersIndex = i;
        }
        else if (axisName.find(eyes_vergence_name) != std::string::npos)
        {
            m_eyeVergIndex = i;
        }
        else if (axisName.find(eyes_tilt_name) != std::string::npos)
        {
            m_eyeTiltIndex = i;
        }
    }

    if (m_eyeVersIndex < 0)
    {
        yError() << "[GazeRetargeting::configure] Failed to find the version joint with the name" << eyes_version_name << "among the neck joints." ;
        return false;
    }
    if (m_eyeVergIndex < 0)
    {
        yError() << "[GazeRetargeting::configure] Failed to find the vergence joint with the name" << eyes_vergence_name << "among the neck joints." ;
        return false;
    }
    if (m_eyeTiltIndex < 0)
    {
        yError() << "[GazeRetargeting::configure] Failed to find the tilt joint with the name" << eyes_tilt_name << "among the neck joints." ;
        return false;
    }

    if (!m_VRInterface.configure(rf))
    {
        return false;
    }

    homeEyes();

    setEyeControlMode(VOCAB_CM_VELOCITY);

    m_configured = true;

    return true;
}

void GazeRetargeting::setEyeGazeAxes(const iDynTree::Axis &leftGaze, const iDynTree::Axis &rightGaze)
{
    m_leftGaze.setDirection(leftGaze.getDirection());
    m_leftGaze.setOrigin(leftGaze.getOrigin());
    m_rightGaze.setDirection(rightGaze.getDirection());
    m_rightGaze.setOrigin(rightGaze.getOrigin());
    m_gazeSet = true;
}

bool GazeRetargeting::update()
{
    if (!m_VRInterface.isActive())
    {
        return true; //do nothing
    }

    double vergenceSpeed = 0.0, versionSpeed = 0.0, tiltSpeed = 0.0;

    if (m_gazeSet)
    {
        if (!m_VRInterface.computeDesiredEyeVelocities(m_leftGaze, m_rightGaze, vergenceSpeed, versionSpeed, tiltSpeed))
        {
            yError() << "[GazeRetargeting::update] Failed to compute the desired eye velocity.";
            return false;
        }
    }

    if (!setDesiredEyeVelocities(vergenceSpeed, versionSpeed, tiltSpeed))
    {
        yError() << "[GazeRetargeting::update] Failed to set the desired eye velocity.";
        return false;
    }

    if (!updateEyeEncoders())
    {
        yError() << "[GazeRetargeting::update] Failed to get eye encoders.";
        return false;
    }

    m_VRInterface.setVRImagesPose(m_eyeVergInRad, m_eyeVersInRad, m_eyeTiltInRad);

    return true;
}

void GazeRetargeting::close()
{
    m_VRInterface.close();
    setEyeControlMode(VOCAB_CM_POSITION);
    homeEyes();
    m_eyesDriver.close();
    m_eyesVel = nullptr;
    m_eyesPos = nullptr;
    m_eyesMode = nullptr;
    m_eyesEnc = nullptr;
    m_configured = false;
}

bool GazeRetargeting::VRInterface::getValueFromRPC(const std::string &query, yarp::os::Value &value)
{
    yarp::os::Bottle cmd, reply;
    cmd.addVocab32(yarp::os::Vocab32::encode(query));
    bool okWrite = m_VRDeviceRPCOutputPort.write(cmd, reply);

    if (!okWrite || reply.size() == 0)
    {
        return false;
    }

    value = reply.get(0);

    return true;
}

bool GazeRetargeting::VRInterface::getValueFromRPC(const std::string &query, bool &value)
{
    yarp::os::Value output;
    if (!getValueFromRPC(query, output) || !output.isBool())
    {
        return false;
    }

    value = output.asBool();

    return true;
}

bool GazeRetargeting::VRInterface::getValueFromRPC(const std::string &query, double &value)
{
    yarp::os::Value output;
    if (!getValueFromRPC(query, output) || !output.isFloat64())
    {
        return false;
    }

    value = output.asFloat64();

    return true;
}

bool GazeRetargeting::VRInterface::getValueFromRPC(const std::string &query, std::string &value)
{
    yarp::os::Value output;
    if (!getValueFromRPC(query, output) || !output.isString())
    {
        return false;
    }

    value = output.asString();

    return true;
}

bool GazeRetargeting::VRInterface::configure(yarp::os::ResourceFinder &rf)
{
    m_name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    std::string VRDeviceRPCOutputPort = rf.check("VRDeviceRPCOutputPortName", yarp::os::Value("/VR/rpc:o")).asString();

    if (!m_VRDeviceRPCOutputPort.open("/" + m_name + VRDeviceRPCOutputPort))
    {
        yError() << "[GazeRetargeting::configure] Failed to open /" + m_name + VRDeviceRPCOutputPort + " port.";
        return false;
    }

    return true;
}

void GazeRetargeting::VRInterface::setVRImagesPose(double vergenceInRad, double versionInRad, double tiltInRad)
{
    m_leftEye.elevation = tiltInRad;
    m_rightEye.elevation = tiltInRad;

    // See https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-vergence-version/icub-vergence-version/#converting-vergenceversion-to-decoupled-lr
    // In the documentation above, the angle is positive clockwise, while the angles we send are positive anticlockwise
    m_leftEye.azimuth = -(versionInRad + vergenceInRad/2.0);
    m_rightEye.azimuth = -(versionInRad - vergenceInRad/2.0);

    m_leftEye.sendAngles();
    m_rightEye.sendAngles();

}

bool GazeRetargeting::VRInterface::computeDesiredEyeVelocities(const iDynTree::Axis &leftEyeGaze, const iDynTree::Axis &rightEyeGaze,
                                                               double &vergenceSpeed, double &versionSpeed, double &tiltSpeed)
{
    //compute the intersection between the gaze ray and the xy plane of the image
    iDynTree::Vector2 leftImageIntersection, rightImageIntersection;
    bool okL = m_leftEye.intersectionInImage(leftEyeGaze, leftImageIntersection);
    bool okR = m_rightEye.intersectionInImage(rightEyeGaze, rightImageIntersection);
    if (!okL || !okR)
    {
        yError() << "[GazeRetargeting::VRInterface::computeDesiredEyeVelocities] Failed to compute the intersection between the gaze and the images.";
        return false;
    }


    //Compute the single eye velocity according to the error (if the error is small, do nothing)
    //Compute the dual eye velocity
    //Saturate the dual eye velocity also according to the kinematic limits (tanh trick)

    return true;
}

bool GazeRetargeting::VRInterface::isActive()
{
    if (m_isActive)
    {
        return true;
    }

    if ((yarp::os::Time::now() - m_lastActiveCheck) < 1.0)
    {
        return false;
    }

    if (m_VRDeviceRPCOutputPort.getOutputCount() == 0)
    {
        yInfo() << "[GazeRetargeting::VRInterface::isActive] The RPC port has not been connected yet to the VR device...";
        return false;
    }

    bool leftEyeActive{false}, rightEyeActive{false};

    if (!getValueFromRPC("isLeftEyeActive", leftEyeActive) || !leftEyeActive)
    {
        yInfo() << "[GazeRetargeting::VRInterface::isActive] The left eye is still not active...";
        return false;
    }

    if (!getValueFromRPC("isRightEyeActive", rightEyeActive) || !rightEyeActive)
    {
        yInfo() << "[GazeRetargeting::VRInterface::isActive] The right eye is still not active...";
        return false;
    }

    double ipd = 0.0;

    if (!getValueFromRPC("getIPD", ipd))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to retrieve the IPD from the VR device.";
        return false;
    }

    m_leftEye.eyePosition.zero();
    m_leftEye.eyePosition(0) = -ipd/2.0;
    m_rightEye.eyePosition.zero();
    m_rightEye.eyePosition(0) = ipd/2.0;

    double eyesZPosition = -1.0;

    if (!getValueFromRPC("getEyesZPosition", eyesZPosition))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to retrieve the eyes Z position from the VR device.";
        return false;
    }

    m_leftEye.imageRelativePosition.zero();
    m_leftEye.imageRelativePosition(2) = eyesZPosition;
    m_rightEye.imageRelativePosition.zero();
    m_rightEye.imageRelativePosition(2) = eyesZPosition;

    std::string leftEyePortName;


    if (!getValueFromRPC("getLeftImageControlPortName", leftEyePortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to retrieve the left image control port name from the VR device.";
        return false;
    }

    std::string leftEyeOutputPortName = m_name + "/leftEye/control:o";

    if (!m_leftEye.controlPort.open(leftEyeOutputPortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to open the port" << leftEyeOutputPortName;
        return false;
    }

    if (!yarp::os::Network::connect(leftEyeOutputPortName, leftEyePortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to connect the port" << leftEyeOutputPortName << "to" << leftEyePortName;
        return false;
    }

    std::string rightEyePortName;


    if (!getValueFromRPC("getRightImageControlPortName", rightEyePortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to retrieve the right image control port name from the VR device.";
        return false;
    }

    std::string rightEyeOutputPortName = m_name + "/rightEye/control:o";

    if (!m_rightEye.controlPort.open(rightEyeOutputPortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to open the port" << rightEyeOutputPortName;
        return false;
    }

    if (!yarp::os::Network::connect(rightEyeOutputPortName, rightEyePortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to connect the port" << rightEyeOutputPortName << "to" << rightEyePortName;
        return false;
    }

    setVRImagesPose(0.0, 0.0, 0.0);

    m_isActive = true;
    return true;;
}

void GazeRetargeting::VRInterface::close()
{
    m_VRDeviceRPCOutputPort.close();
    m_leftEye.close();
    m_rightEye.close();
}

void GazeRetargeting::VRInterface::EyeControl::sendAngles()
{
    yarp::sig::Vector& output = controlPort.prepare();

    output.resize(2);
    output(0) = azimuth;
    output(1) = elevation;

    controlPort.write();
}

iDynTree::Transform GazeRetargeting::VRInterface::EyeControl::currentImageTransform()
{
    //The X axis is pointing to the right in the VIEW space, the Y axis is pointing upwards in the VIEW space
    iDynTree::Rotation rotation = iDynTree::Rotation::RotX(elevation) * iDynTree::Rotation::RotY(azimuth);
    iDynTree::Position position = rotation * imageRelativePosition + eyePosition;

    return iDynTree::Transform(rotation, position);
}

bool GazeRetargeting::VRInterface::EyeControl::intersectionInImage(const iDynTree::Axis &gazeInHeadsetFrame, iDynTree::Vector2& output)
{
    output.zero();

    iDynTree::Axis gazeInImage = currentImageTransform().inverse() * gazeInHeadsetFrame;

    const iDynTree::Position& origin = gazeInImage.getOrigin();
    const iDynTree::Direction& direction = gazeInImage.getDirection();

    if (std::abs(direction(2)) < 1e-15)
    {
        return false;
    }

    double alpha = -origin(2) / direction(2);

    output(0) = origin(0) + alpha * direction(0);
    output(1) = origin(1) + alpha * direction(1);

    return true;

}

void GazeRetargeting::VRInterface::EyeControl::close()
{
    controlPort.close();
}
