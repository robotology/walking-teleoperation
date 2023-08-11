// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <SRanipalModule.hpp>
#include <string>
#include <yarp/os/LogStream.h>

bool SRanipalModule::configure(yarp::os::ResourceFinder &rf)
{
    yInfo() << "Configuring the SRanipalModule with options" << rf.toString();

    std::string name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    setName(name.c_str());
    std::string robot = rf.check("robot", yarp::os::Value("icub"), "The name of the robot to connect to.").asString();

    m_useEyebrows = !rf.check("noEyebrows") || (!rf.find("noEyebrows").isNull() && !rf.find("noEyebrows").asBool()); //True if noEyebrows is not set or set to false
    m_useLip = !rf.check("noLip") || (!rf.find("noLip").isNull() && !rf.find("noLip").asBool()); //True if noLip is not set or set to false
    m_useEyelids = !rf.check("noEyelids") || (!rf.find("noEyelids").isNull() && !rf.find("noEyelids").asBool()); //True if noEyelids is not set or set to false
    m_useGaze = !rf.check("noGaze") || (!rf.find("noGaze").isNull() && !rf.find("noGaze").asBool()); // True if noGaze is not set or set to false
    m_useEyeExpressions = !rf.check("noEyeExpressions") || (!rf.find("noEyeExpressions").isNull() && !rf.find("noEyeExpressions").asBool()); //True if noEyeExpressions is not set or set to false
    m_useAdvancedJoypad = m_advancedJoypad.enabled(rf);

    double defaultPeriod = 0.1;

    if (m_useEyebrows)
    {
        yInfo() << "[SRanipalModule::configure] Controlling the eyebrows.";
    }
    else
    {
        yInfo() << "[SRanipalModule::configure] Skipping eyebrows control.";
    }

    if (m_useEyebrows || m_useLip || m_useEyeExpressions)
    {
        if (!m_faceExpressions.configure(rf))
        {
            yError()
                << "[SRanipalModule::configure] Failed to initialize face expression retargeting.";
            return false;
        }
    }

    if (m_useGaze || m_useAdvancedJoypad)
    {
        m_VRInterface = std::make_shared<VRInterface>();

        if (!m_VRInterface->configure(rf))
        {
            yError() << "[SRanipalModule::configure] Failed to configure VR interface.";
            return false;
        }
    }

    if (m_useEyebrows || m_useEyelids || m_useGaze || m_useAdvancedJoypad || m_useEyeExpressions)
    {
        if (m_useEyelids)
        {
            if (!m_eyelidsRetargeting.configure(rf))
            {
                yError() << "[SRanipalModule::configure] Failed to configure the eyelids.";
                return false;
            }

            if (m_eyelidsRetargeting.usingEylidsVelocityControl())
            {
                defaultPeriod = 0.01; //If we control the eyelids in velocity control, we use faster loops.
            }
            yInfo() << "[SRanipalModule::configure] Controlling the eyelids.";
        }
        else
        {
            yInfo() << "[SRanipalModule::configure] Skipping eyelids control.";
        }

        if (m_useGaze)
        {
            if (!m_gazeRetargeting.configure(rf, m_VRInterface))
            {
                yError() << "[SRanipalModule::configure] Failed to configure the gaze retargeting.";
            return false;
            }

            defaultPeriod = 0.01; //Since we use velocity control for the gaze, we use faster loops
            yInfo() << "[SRanipalModule::configure] Controlling the gaze.";
        }
        else
        {
            yInfo() << "[SRanipalModule::configure] Skipping gaze control.";
        }

        if (m_useAdvancedJoypad)
        {
            if (!m_advancedJoypad.configure(rf, m_VRInterface))
            {
                yError() << "[SRanipalModule::configure] Failed to configure the advanced joypad.";
                return false;
            }
            yInfo() << "[SRanipalModule::configure] Using the advanced joypad.";
        }
        else
        {
            yInfo() << "[SRanipalModule::configure] Skipping advanced joypad.";
        }

        if (!m_sranipalInterface.initializeEyeEngine())
        {
            return false;
        }
    }

    if (m_useLip)
    {
        if (!m_sranipalInterface.initializeLipEngine())
        {
            return false;
        }

        std::string lipImageOutputPort = rf.check("lipImagePortName", yarp::os::Value("/lipImage:o"), "The name of the output port for the lip camera.").asString();
        if (!m_lipImagePort.open("/" + name + lipImageOutputPort))
        {
            yError() << "[SRanipalModule::configure] Failed to open /" + name + lipImageOutputPort + " port.";
            return false;
        }

        yInfo() << "[SRanipalModule::configure] Using Lip tracking.";
    }
    else
    {
        yInfo() << "[SRanipalModule::configure] Skipping lip control.";
    }

    m_period = rf.check("period", yarp::os::Value(defaultPeriod)).asFloat64();

    if (m_useEyebrows || m_useEyelids || m_useGaze || m_useAdvancedJoypad || m_useEyeExpressions) //Run the eye calibration as last thing
    {
        bool skipEyeCalibration = rf.check("skipEyeCalibration") && (rf.find("skipEyeCalibration").isNull() || rf.find("skipEyeCalibration").asBool());
        bool forceEyeCalibration = rf.check("forceEyeCalibration") && (rf.find("forceEyeCalibration").isNull() || rf.find("forceEyeCalibration").asBool());

        if (forceEyeCalibration && skipEyeCalibration)
        {
            yError() << "[SRanipalModule::configure] Both skipEyeCalibration and forceEyeCalibration are set!";
            return false;
        }

        if (!skipEyeCalibration) // If skipCalibration is not set or set to false
        {
            bool needCalibration = false;
            if (!m_sranipalInterface.isEyeCalibrationNeeded(needCalibration))
            {
                return false;
            }

            if (needCalibration || forceEyeCalibration)
            {
                yInfo() << "[SRanipalModule::configure] Runnning Eye calibration";
                if (!m_sranipalInterface.calibrateEyeTracking())
                {
                    return false;
                }
            }
        }
    }

    yInfo() << "SRanipalModule started correctly.";

    return true;
}

double SRanipalModule::getPeriod()
{
    return m_period;
}

bool SRanipalModule::updateModule()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if ((m_useEyebrows || m_useEyelids || m_useGaze || m_useAdvancedJoypad || m_useEyeExpressions) && m_sranipalInterface.updateEyeData()) {

        double eyeWideness{0.0};
        if (m_useEyebrows && m_sranipalInterface.getEyeWideness(eyeWideness))
        {
            m_faceExpressions.updateEyebrows(eyeWideness);
        }

        double left_eye_openness, right_eye_openness;
        if ((m_useEyelids || m_useAdvancedJoypad || m_useEyeExpressions) && m_sranipalInterface.getEyeOpenness(left_eye_openness, right_eye_openness))
        {
            if (m_useEyelids)
            {
                m_eyelidsRetargeting.setDesiredEyeOpennes(left_eye_openness, right_eye_openness);
            }

            if (m_useAdvancedJoypad)
            {
                m_advancedJoypad.setEyeOpenness(left_eye_openness, right_eye_openness);
            }

            if (m_useEyeExpressions)
            {
                m_faceExpressions.updateEyeExpressions(left_eye_openness, right_eye_openness);
            }
        }

        iDynTree::Axis leftGaze, rightGaze;
        if (m_useGaze && m_sranipalInterface.getGazeAxes(leftGaze, rightGaze))
        {
            m_gazeRetargeting.setOperatorEyeGazeAxes(leftGaze, rightGaze);
        }
    }

    if (m_useEyelids)
    {
        m_eyelidsRetargeting.update();
    }

    if (m_useGaze)
    {
        m_gazeRetargeting.update();
    }

    if (m_useAdvancedJoypad)
    {
        m_advancedJoypad.update();
    }

    SRanipalInterface::LipExpressions lipExpressions;
    if (m_useLip && m_sranipalInterface.updateLipData() && m_sranipalInterface.getLipExpressions(lipExpressions))
    {
        m_faceExpressions.updateLip(lipExpressions);

        yarp::sig::FlexImage& outputImage = m_lipImagePort.prepare();
        outputImage.setPixelCode(VOCAB_PIXEL_MONO);
        outputImage.setExternal(m_sranipalInterface.lipImage(), 800, 400);
        m_lipImagePort.write();
    }

    return true;
}

bool SRanipalModule::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_sranipalInterface.close();
    m_eyelidsRetargeting.close();
    m_gazeRetargeting.close();
    m_faceExpressions.close();
    m_advancedJoypad.close();
    if (m_VRInterface)
    {
        m_VRInterface->close();
        m_VRInterface = nullptr;
    }
    m_lipImagePort.close();
    yInfo() << "Closing";
    return true;
}

