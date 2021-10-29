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
#include <yarp/os/Time.h>

void GazeRetargeting::setEyeControlMode(int controlMode)
{
    if (m_eyesMode)
    {
        int eyeAxis[] = {m_eyeTiltIndex, m_eyeVersIndex, m_eyeVergIndex};
        int controlModes[] = {controlMode, controlMode, controlMode};
        m_eyesMode->setControlModes(3, eyeAxis, controlModes);
    }
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

    setEyeControlMode(VOCAB_CM_VELOCITY);

    m_configured = true;

    return true;
}

bool GazeRetargeting::update()
{
    //I could check if left and right images are active, if not, skip until the first time they are active
    //the first time they are active, ask from RPC the offset of the eyes.

    //get the eyes encoders
    //compute the single eye angles
    //set the eye angles in the port to command the display position

    //get current left and right eye gaze direction and origin
    //get left and right image pose from transform server
    //transform the gaze rays in the image frame
    //compute the intersection between the gaze ray and the xy plane of the image
    //compute the desired aiming point according to the eyes offset
    //Get the errors between the current intersection and the desired aim point
    //Compute the single eye velocity according to the error (if the error is small, do nothing)
    //Compute the dual eye velocity
    //Set the dual eye velocity to the robot

    return true;
}

void GazeRetargeting::close()
{
    m_VRInterface.close();
    setEyeControlMode(VOCAB_CM_POSITION);
    m_eyesDriver.close();
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

    if (!getValueFromRPC("getIPD", m_IPD))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to retrieve the IPD from the VR device.";
        return false;
    }


    if (!getValueFromRPC("getEyesZPosition", m_eyesZPosition))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to retrieve the eyes Z position from the VR device.";
        return false;
    }

    std::string leftEyePortName;


    if (!getValueFromRPC("getLeftImageControlPortName", leftEyePortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to retrieve the left image control port name from the VR device.";
        return false;
    }

    std::string leftEyeOutputPortName = m_name + "/leftEye/control:o";

    if (!m_leftEyeControlPort.open(leftEyeOutputPortName))
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

    if (!m_rightEyeControlPort.open(rightEyeOutputPortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to open the port" << rightEyeOutputPortName;
        return false;
    }

    if (!yarp::os::Network::connect(rightEyeOutputPortName, rightEyePortName))
    {
        yError() << "[GazeRetargeting::VRInterface::isActive] Failed to connect the port" << rightEyeOutputPortName << "to" << rightEyePortName;
        return false;
    }

    m_isActive = true;
    return true;;
}

void GazeRetargeting::VRInterface::close()
{
    m_VRDeviceRPCOutputPort.close();
    m_leftEyeControlPort.close();
    m_rightEyeControlPort.close();
}
