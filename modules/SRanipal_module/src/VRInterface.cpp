// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#ifndef _USE_MATH_DEFINES //for using M_PI
#define _USE_MATH_DEFINES
#endif

#include <yarp/os/LogStream.h>
#include <VRInterface.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Axis.h>

bool VRInterface::getValueFromRPC(const std::string &query, yarp::os::Value &value)
{
    yarp::os::Bottle cmd, reply;
    cmd.addString(query);
    bool okWrite = m_VRDeviceRPCOutputPort.write(cmd, reply);
    yDebug() << "[VRInterface::getValueFromRPC] Sent the following command to RPC:"
             << query;

    if (!okWrite || reply.size() == 0)
    {
        yDebug()
            << "[VRInterface::getValueFromRPC] Failed to get an answer. (okWrite ="
            << okWrite << "reply.size()" << reply.size();
        return false;
    }

    yDebug() << "[VRInterface::getValueFromRPC] Received answer:" << reply.toString();

    value = reply.get(0);

    return true;
}

bool VRInterface::getValueFromRPC(const std::string &query, bool &value)
{
    yarp::os::Value output;
    if (!getValueFromRPC(query, output))
    {
        return false;
    }

    if (output.isVocab32())
    {
        value = yarp::os::Vocab32::decode(output.asVocab32()).find("ok") != std::string::npos;
    }
    else if (output.isBool())
    {
        value = output.asBool();
    }
    else if (output.isString())
    {
        value = output.asString().find("ok") != std::string::npos;
    }
    else
    {
        return false;
    }

    return true;
}

bool VRInterface::getValueFromRPC(const std::string &query, double &value)
{
    yarp::os::Value output;
    if (!getValueFromRPC(query, output) || !output.isFloat64())
    {
        return false;
    }

    value = output.asFloat64();

    return true;
}

bool VRInterface::getValueFromRPC(const std::string &query, std::string &value)
{
    yarp::os::Value output;
    if (!getValueFromRPC(query, output) || !output.isString())
    {
        return false;
    }

    value = output.asString();

    return true;
}

iDynTree::Vector2 VRInterface::applyDeadzone(const iDynTree::Vector2 &input)
{
    iDynTree::Vector2 output;
    output.zero();

    Eigen::Map<const Eigen::Vector2d> map = iDynTree::toEigen(input);
    Eigen::Map<Eigen::Vector2d> outputMap = iDynTree::toEigen(output);

    double inputNorm = map.norm();
    bool deadzoneNotActiveAndErrorStillHigh = !m_deadzoneActive && inputNorm > m_errorDeadzone; //True if the deadzone is not active, and the error is too high to activate it
    bool deadzoneActiveButErrorVeryHigh = m_deadzoneActive && inputNorm > m_errorDeadzoneActivation; //True if the deadzone is active, but the error is bigger than the threshold to deactivate it
    bool timeCheckNotActive = m_deadzoneMinActivationTimeInS <= 0.0; //if the parameter m_deadzoneMinActivationTimeInS is lower or equal than zero it means that we should not consider the time check when deactivating the deadzone
    bool timeCheckPerformedOnce = m_deadzoneActivationTime >= 0.0; //m_deadzoneActivationTime contains the first time instant in which deadzoneActiveButErrorVeryHigh becomes true. If it is greater or equal than zero, it means that it has been set already.
    bool enoughTimePassed =  timeCheckNotActive || (timeCheckPerformedOnce && ((yarp::os::Time::now() - m_deadzoneActivationTime) >= m_deadzoneMinActivationTimeInS)); //enoughTimePassed is true either if we don't use any time threshold, or when the time condition has triggered.

    if ((deadzoneActiveButErrorVeryHigh && enoughTimePassed) || deadzoneNotActiveAndErrorStillHigh)
    {
        m_deadzoneActive = false;
        m_deadzoneActivationTime = -1.0;
        outputMap = (1.0 - m_errorDeadzone / inputNorm) * map;
        return output;
    }

    m_deadzoneActive = true;

    if (deadzoneActiveButErrorVeryHigh && !timeCheckPerformedOnce) //It is the the first time that the gaze exits the deadzone
    {
        m_deadzoneActivationTime = yarp::os::Time::now();
    }

    if (!deadzoneActiveButErrorVeryHigh) //The gaze is still in the deadzone
    {
        m_deadzoneActivationTime = -1.0;
    }

    return output;
}

double VRInterface::applyQuantization(double input, double quantization)
{
    int level = static_cast<int>(std::round(input / quantization));
    return level * quantization;
}

bool VRInterface::configure(const yarp::os::ResourceFinder &rf)
{
    m_name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    std::string VRDeviceRPCOutputPort = rf.check("VRDeviceRPCOutputPortName", yarp::os::Value("/VR/rpc:o")).asString();

    if (!m_VRDeviceRPCOutputPort.open("/" + m_name + VRDeviceRPCOutputPort))
    {
        yError() << "[configure] Failed to open /" + m_name + VRDeviceRPCOutputPort + " port.";
        return false;
    }

    m_velocityGain = rf.check("gazeVelocityGain", yarp::os::Value(2.0)).asFloat64();
    m_errorDeadzone = rf.check("gazeDeadzone", yarp::os::Value(0.02)).asFloat64();
    double activationOffset = rf.check("gazeDeadzoneActivationOffset", yarp::os::Value(0.1)).asFloat64();

    if (activationOffset < 0)
    {
        yError() << "[configure] gazeDeadzoneActivationOffset is supposed to be non-negative.";
        return false;
    }

    m_errorDeadzoneActivation = m_errorDeadzone + activationOffset;

    m_deadzoneMinActivationTimeInS = rf.check("gazeDeadzoneMinActivationTime", yarp::os::Value(0.5)).asFloat64();

    double gazeAccuracyInDeg = rf.check("gazeMovementAccuracyInDeg", yarp::os::Value(0.1)).asFloat64();

    if (gazeAccuracyInDeg <= 0.0)
    {
        yError() << "[configure] gazeAccuracyInDeg is supposed to be strictly positive.";
        return false;
    }

    m_eyeMovementAccuracyInRad = iDynTree::deg2rad(gazeAccuracyInDeg);

    return true;
}

void VRInterface::setVRImagesPose(double vergenceInRad, double versionInRad, double tiltInRad)
{
    m_leftEye.elevation = tiltInRad;
    m_rightEye.elevation = tiltInRad;

    // See https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-vergence-version/icub-vergence-version/#converting-vergenceversion-to-decoupled-lr
    // In the documentation above, the angle is positive clockwise, while the angles we send are positive anticlockwise
    m_leftEye.azimuth = -(versionInRad + vergenceInRad/2.0);
    m_rightEye.azimuth = -(versionInRad - vergenceInRad/2.0);

    m_leftEye.elevation  = applyQuantization(m_leftEye.elevation,  m_eyeMovementAccuracyInRad);
    m_leftEye.azimuth    = applyQuantization(m_leftEye.azimuth,    m_eyeMovementAccuracyInRad);
    m_rightEye.elevation = applyQuantization(m_rightEye.elevation, m_eyeMovementAccuracyInRad);
    m_rightEye.azimuth   = applyQuantization(m_rightEye.azimuth,   m_eyeMovementAccuracyInRad);

    m_leftEye.sendAngles();
    m_rightEye.sendAngles();

}

bool VRInterface::computeDesiredRobotEyeVelocities(const iDynTree::Axis &operatorLeftEyeGaze, const iDynTree::Axis &operatorRightEyeGaze,
                                                               double &vergenceSpeedInRadS, double &versionSpeedInRadS, double &tiltSpeedInRadS)
{
    //compute the intersection between the gaze ray and the xy plane of the image
    iDynTree::Vector2 leftImageIntersection, rightImageIntersection;
    bool okL = m_leftEye.intersectionInImage(operatorLeftEyeGaze, leftImageIntersection);
    bool okR = m_rightEye.intersectionInImage(operatorRightEyeGaze, rightImageIntersection);
    if (!okL || !okR)
    {
        yError() << "[VRInterface::computeDesiredRobotEyeVelocities] Failed to compute the intersection between the gaze and the images.";
        return false;
    }

    //First, apply a deadzone on the intersection with the image to avoid unwanted motions
    leftImageIntersection = applyDeadzone(leftImageIntersection);
    rightImageIntersection = applyDeadzone(rightImageIntersection);

    double leftEyeDistance = iDynTree::toEigen(m_leftEye.imageRelativePosition).norm();
    double rightEyeDistance = iDynTree::toEigen(m_rightEye.imageRelativePosition).norm();

    if (leftEyeDistance < 1e-10)
    {
        yError() << "[VRInterface::computeDesiredRobotEyeVelocities] The left image position "
                    "is too close to the eye.";
        return false;
    }

    if (rightEyeDistance < 1e-10)
    {
        yError() << "[VRInterface::computeDesiredRobotEyeVelocities] The right image position "
                    "is too close to the eye.";
        return false;
    }

    //Compute the desired single eye velocity
    double leftElevationVelocity, rightElevationVelocity, leftAzimuthVelocity, rightAzimuthVelocity;
    leftElevationVelocity = m_velocityGain * leftImageIntersection(1) / leftEyeDistance; //The Y axis is pointing upward. If the operator is intersecting the image above the center, moves the eye up
    rightElevationVelocity = m_velocityGain * rightImageIntersection(1) / rightEyeDistance;

    leftAzimuthVelocity = -m_velocityGain * leftImageIntersection(0) / leftEyeDistance; //The X axis is pointing to the right. If the operator is looking on the right of the center, moves the eye clockwise (hence the minus sign).
    rightAzimuthVelocity = -m_velocityGain * rightImageIntersection(0) / rightEyeDistance;

    //Compute the dual eye velocity
    tiltSpeedInRadS = 0.5 * (leftElevationVelocity + rightElevationVelocity); //Ideally they should be equal. Set the desired velocity to the average value.
    //Compute the version and vergence velocity according to https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-vergence-version/icub-vergence-version
    double leftVersVelocity = -leftAzimuthVelocity; //According to https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-vergence-version/icub-vergence-version, positive version is clockwise, while positive azimuth is anticlockwise
    double rightVersVelocity = -rightAzimuthVelocity;
    versionSpeedInRadS = 0.5 * (leftVersVelocity + rightVersVelocity);
    vergenceSpeedInRadS = leftVersVelocity - rightVersVelocity;

    return true;
}

bool VRInterface::setGUIEnabled(int index, bool enabled)
{
    if (!m_isActive)
    {
        yInfo() << "[VRInterface::setGUIEnabled] The VR interface is not active yet.";
        return false;
    }

    yarp::os::Bottle cmd, reply;
    cmd.addString("setGUIEnabled");
    cmd.addInt32(index);
    cmd.add(yarp::os::Value(enabled));
    bool okWrite = m_VRDeviceRPCOutputPort.write(cmd, reply);
    yDebug() << "[VRInterface::setGUIEnabled] Sent the following command to RPC:"
             << cmd.toString();

    if (!okWrite || reply.size() == 0)
    {
        yDebug()
            << "[VRInterface::setGUIEnabled] Failed to get an answer. (okWrite ="
            << okWrite << "reply.size()" << reply.size();
        return false;
    }

    yDebug() << "[VRInterface::setGUIEnabled] Received answer:" << reply.toString();

    yarp::os::Value value = reply.get(0);

    if (value.isVocab32())
    {
        return yarp::os::Vocab32::decode(value.asVocab32()).find("ok") != std::string::npos;
    }
    else if (value.isBool())
    {
        return value.asBool();
    }
    else if (value.isString())
    {
        return value.asString().find("ok") != std::string::npos;
    }

    return false;
}

bool VRInterface::isActive()
{
    if (m_isActive)
    {
        return true;
    }

    if ((yarp::os::Time::now() - m_lastActiveCheck) < 1.0) //Check once per second
    {
        return false;
    }
    m_lastActiveCheck = yarp::os::Time::now();

    if (m_VRDeviceRPCOutputPort.getOutputCount() == 0)
    {
        yInfo() << "[VRInterface::isActive] The RPC port has not been connected yet to the VR device...";
        return false;
    }

    bool leftEyeActive{false}, rightEyeActive{false};

    if (!getValueFromRPC("isLeftEyeActive", leftEyeActive) || !leftEyeActive)
    {
        yInfo() << "[VRInterface::isActive] The left eye is still not active...";
        return false;
    }

    if (!getValueFromRPC("isRightEyeActive", rightEyeActive) || !rightEyeActive)
    {
        yInfo() << "[VRInterface::isActive] The right eye is still not active...";
        return false;
    }

    double interCameraDistance = 0.0;

    if (!getValueFromRPC("getInterCameraDistance", interCameraDistance))
    {
        yError() << "[VRInterface::isActive] Failed to retrieve the inter camera distance from the VR device.";
        return false;
    }

    m_leftEye.eyePosition.zero();
    m_leftEye.eyePosition(0) = -interCameraDistance/2.0;
    m_rightEye.eyePosition.zero();
    m_rightEye.eyePosition(0) = interCameraDistance/2.0;

    double eyesZPosition = -1.0;

    if (!getValueFromRPC("getEyesZPosition", eyesZPosition))
    {
        yError() << "[VRInterface::isActive] Failed to retrieve the eyes Z position from the VR device.";
        return false;
    }

    m_leftEye.imageRelativePosition.zero();
    m_leftEye.imageRelativePosition(2) = eyesZPosition;
    m_rightEye.imageRelativePosition.zero();
    m_rightEye.imageRelativePosition(2) = eyesZPosition;

    std::string leftEyePortName;

    if (!getValueFromRPC("getLeftImageControlPortName", leftEyePortName))
    {
        yError() << "[VRInterface::isActive] Failed to retrieve the left image control port name from the VR device.";
        return false;
    }

    std::string leftEyeOutputPortName = "/" + m_name + "/leftEye/control:o";

    if (!m_leftEye.imageControlPort.open(leftEyeOutputPortName))
    {
        yError() << "[VRInterface::isActive] Failed to open the port" << leftEyeOutputPortName;
        return false;
    }

    if (!yarp::os::Network::connect(leftEyeOutputPortName, leftEyePortName))
    {
        yError() << "[VRInterface::isActive] Failed to connect the port" << leftEyeOutputPortName << "to" << leftEyePortName;
        return false;
    }

    std::string rightEyePortName;


    if (!getValueFromRPC("getRightImageControlPortName", rightEyePortName))
    {
        yError() << "[VRInterface::isActive] Failed to retrieve the right image control port name from the VR device.";
        return false;
    }

    std::string rightEyeOutputPortName = "/" + m_name + "/rightEye/control:o";

    if (!m_rightEye.imageControlPort.open(rightEyeOutputPortName))
    {
        yError() << "[VRInterface::isActive] Failed to open the port" << rightEyeOutputPortName;
        return false;
    }

    if (!yarp::os::Network::connect(rightEyeOutputPortName, rightEyePortName))
    {
        yError() << "[VRInterface::isActive] Failed to connect the port" << rightEyeOutputPortName << "to" << rightEyePortName;
        return false;
    }

    setVRImagesPose(0.0, 0.0, 0.0);

    m_isActive = true;

    yInfo() << "[VRInterface::isActive] VRInterface ready!";

    return true;;
}

void VRInterface::close()
{
    m_VRDeviceRPCOutputPort.close();
    m_leftEye.close();
    m_rightEye.close();
}

void VRInterface::EyeControl::sendAngles()
{
    yarp::sig::Vector& output = imageControlPort.prepare();

    output.resize(2);
    output(0) = azimuth;
    output(1) = elevation;

    imageControlPort.write();
}

iDynTree::Transform VRInterface::EyeControl::currentImageTransform()
{
    //The X axis is pointing to the right in the VIEW space, the Y axis is pointing upwards in the VIEW space
    iDynTree::Rotation rotation = iDynTree::Rotation::RotX(elevation) * iDynTree::Rotation::RotY(azimuth);
    iDynTree::Position position = rotation * imageRelativePosition + eyePosition;

    return iDynTree::Transform(rotation, position);
}

bool VRInterface::EyeControl::intersectionInImage(const iDynTree::Axis &operatorGazeInSRanipalFrame, iDynTree::Vector2& output)
{
    output.zero();

    iDynTree::Transform headsetToSranipalTransform(iDynTree::Rotation::RotY(M_PI), //The frame in which gazeInSRanipalFrame has the Y pointing up and the Z forward. The headset frame has the Z backward
                                                   iDynTree::Position::Zero());    //See https://www.researchgate.net/figure/Coordinate-system-of-HTC-VIVE-Pro-Eye-based-on-the-manual-of-SRanipal-SDK-A_fig1_346058398

    iDynTree::Axis gazeInHeadsetFrame = headsetToSranipalTransform * operatorGazeInSRanipalFrame;

    iDynTree::Axis gazeInImage
        = currentImageTransform().inverse() * gazeInHeadsetFrame;

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

void VRInterface::EyeControl::close()
{
    azimuth = 0.0;
    elevation = 0.0;
    sendAngles();
    imageControlPort.close();
}
