/**
 * @file SRanipalModule.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <SRanipalModule.hpp>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <yarp/os/LogStream.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits.h>
#include <SRanipal.h>
#include <SRanipal_Eye.h>
#include <SRanipal_Lip.h>
#include <SRanipal_Enums.h>
#include <SRanipal_NotRelease.h>

const char * SRanipalModule::errorCodeToString(int error) const
{
    using namespace ViveSR;
    switch (error) {
    case(RUNTIME_NOT_FOUND):     return "RUNTIME_NOT_FOUND";
    case(NOT_INITIAL):           return "NOT_INITIAL";
    case(FAILED):                return "FAILED";
    case(WORK):                  return "WORK";
    case(INVALID_INPUT):         return "INVALID_INPUT";
    case(FILE_NOT_FOUND):        return "FILE_NOT_FOUND";
    case(DATA_NOT_FOUND):        return "DATA_NOT_FOUND";
    case(UNDEFINED):             return "UNDEFINED";
    case(INITIAL_FAILED):        return "INITIAL_FAILED";
    case(NOT_IMPLEMENTED):       return "NOT_IMPLEMENTED";
    case(NULL_POINTER):          return "NULL_POINTER";
    case(OVER_MAX_LENGTH):       return "OVER_MAX_LENGTH";
    case(FILE_INVALID):          return "FILE_INVALID";
    case(UNINSTALL_STEAM):       return "UNINSTALL_STEAM";
    case(MEMCPY_FAIL):           return "MEMCPY_FAIL";
    case(NOT_MATCH):             return "NOT_MATCH";
    case(NODE_NOT_EXIST):        return "NODE_NOT_EXIST";
    case(UNKONW_MODULE):         return "UNKONW_MODULE";
    case(MODULE_FULL):           return "MODULE_FULL";
    case(UNKNOW_TYPE):           return "UNKNOW_TYPE";
    case(INVALID_MODULE):        return "INVALID_MODULE";
    case(INVALID_TYPE):          return "INVALID_TYPE";
    case(MEMORY_NOT_ENOUGH):     return "MEMORY_NOT_ENOUGH";
    case(BUSY):                  return "BUSY";
    case(NOT_SUPPORTED):         return "NOT_SUPPORTED";
    case(INVALID_VALUE):         return "INVALID_VALUE";
    case(COMING_SOON):           return "COMING_SOON";
    case(INVALID_CHANGE):        return "INVALID_CHANGE";
    case(TIMEOUT):               return "TIMEOUT";
    case(DEVICE_NOT_FOUND):      return "DEVICE_NOT_FOUND";
    case(INVALID_DEVICE):        return "INVALID_DEVICE";
    case(NOT_AUTHORIZED):        return "NOT_AUTHORIZED";
    case(ALREADY):               return "ALREADY";
    case(INTERNAL):              return "INTERNAL";
    case(CONNECTION_FAILED):     return "CONNECTION_FAILED";
    case(ALLOCATION_FAILED):     return "ALLOCATION_FAILED";
    case(OPERATION_FAILED):      return "OPERATION_FAILED";
    case(NOT_AVAILABLE):         return "NOT_AVAILABLE";
    case(CALLBACK_IN_PROGRESS):  return "CALLBACK_IN_PROGRESS";
    case(SERVICE_NOT_FOUND):     return "SERVICE_NOT_FOUND";
    case(DISABLED_BY_USER):      return "DISABLED_BY_USER";
    case(EULA_NOT_ACCEPT):       return "EULA_NOT_ACCEPT";
    case(RUNTIME_NO_RESPONSE):   return "RUNTIME_NO_RESPONSE";
    case(OPENCL_NOT_SUPPORT):    return "OPENCL_NOT_SUPPORT";
    case(NOT_SUPPORT_EYE_TRACKING): return "NOT_SUPPORT_EYE_TRACKING";
    case(LIP_NOT_SUPPORT):       return "LIP_NOT_SUPPORT";
    case(anipal::Eye::CALIBRATION_IS_ALREADY_RUNNING): return "CALIBRATION_IS_ALREADY_RUNNING";
    case(anipal::Eye::OPENVR_DASHBOARD_ACTIVATED): return "OPENVR_DASHBOARD_ACTIVATED";
    case(anipal::Eye::OPENVR_INIT_FAILED): return "OPENVR_INIT_FAILED";
    case(anipal::Eye::OPENVR_OVERLAY_ALREADY_EXISTS): return "OPENVR_OVERLAY_ALREADY_EXISTS";
    case(anipal::Eye::OPENVR_OVERLAY_CREATE_FAILED): return "OPENVR_OVERLAY_CREATE_FAILED";
    case(anipal::Eye::OPENVR_OVERLAY_INTERFACE_INVALID): return "OPENVR_OVERLAY_INTERFACE_INVALID";
    case(anipal::Eye::OPENVR_QUIT): return "OPENVR_QUIT";
    default:
        return "UNKNOWN";
    }
    return "";
}

void SRanipalModule::sendFaceExpression(const std::string& part, const std::string& emotion)
{
    if (emotion != m_currentExpressions[part])
    {
        yarp::os::Bottle cmd, reply;
        cmd.addVocab(yarp::os::Vocab::encode("set"));
        cmd.addVocab(yarp::os::Vocab::encode(part));
        cmd.addVocab(yarp::os::Vocab::encode(emotion));
        m_emotionsOutputPort.write(cmd, reply);
        m_currentExpressions[part] = emotion;
        yInfo() << "Sending" << emotion << "to" << part;
    }
}

bool SRanipalModule::configure(yarp::os::ResourceFinder &rf)
{
    int output;

    std::string name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    setName(name.c_str());
    std::string robot = rf.check("robot", yarp::os::Value("icub"), "The name of the robot to connect to.").asString();

    std::string emotionsPortOut = rf.check("emotionsOutputPortName", yarp::os::Value("/emotions:o"), "The name of the output port for the emotions.").asString();
    if (!m_emotionsOutputPort.open("/" + name + emotionsPortOut))
    {
        yError() << "[SRanipalModule::configure] Failed to open /" + name + emotionsPortOut + " port.";
        return false;
    }

    m_useEye = !rf.check("noEye") || (!rf.find("noEye").asBool()); //True if noEye is not set or set to false
    m_useLip = !rf.check("noLip") || (!rf.find("noLip").asBool()); //True if noLip is not set or set to false
    m_period = rf.check("period", yarp::os::Value(0.1)).asDouble();
    m_lipExpressionThreshold = rf.check("lipExpressionThreshold", yarp::os::Value(0.2)).asDouble();
    m_eyeWideSurprisedThreshold = rf.check("eyeWideSurprisedThreshold", yarp::os::Value(0.2)).asDouble();

    if (m_useEye)
    {
        output = ViveSR::anipal::Initial(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE_V2, NULL);
        if (output != ViveSR::Error::WORK) {
            if (output == ViveSR::Error::RUNTIME_NOT_FOUND)
            {
                yError() << "[SRanipalModule::configure] SRanipal runtime non installed/available.";
            }
            else if (output == ViveSR::Error::NOT_SUPPORT_EYE_TRACKING)
            {
                yError() << "[SRanipalModule::configure] This HMD do not have eye tracking feature!";
            }
            yError("[SRanipalModule::configure] Failed to initialize Eye engine [%d - %s].", output, errorCodeToString(output));
            return false;
        }

        bool skipEyeCalibration = rf.check("skipEyeCalibration") && (rf.find("skipEyeCalibration").isNull() || rf.find("skipEyeCalibration").asBool());
        bool forceEyeCalibration = rf.check("forceEyeCalibration") && (rf.find("forceEyeCalibration").isNull() || rf.find("forceEyeCalibration").asBool());

        if (forceEyeCalibration && skipEyeCalibration)
        {
            yError() << "Both skipEyeCalibration and forceEyeCalibration are set!";
            return false;
        }

        if (!skipEyeCalibration) //If skipCalibration is not set or set to false
        {
            bool needCalibration = false;
            output = ViveSR::anipal::Eye::IsUserNeedCalibration(&needCalibration);
            if (output != ViveSR::Error::WORK) {
                yError("[SRanipalModule::configure] Failed to initialize check if the eye calibration is needed [%d - %s].", output, errorCodeToString(output));
                return false;
            }

            if (needCalibration || forceEyeCalibration) {
                yInfo() << "[SRanipalModule::configure] Runnning Eye calibration";
                output = ViveSR::anipal::Eye::LaunchEyeCalibration(nullptr);
                if (output != ViveSR::Error::WORK) {
                    yError("[SRanipalModule::configure] Failed to calibrate the eyes [%d - %s].", output, errorCodeToString(output));
                    return false;
                }
            }
        }

        yarp::os::Property rcb_face_conf{{"device", yarp::os::Value("remote_controlboard")},
                                         {"local", yarp::os::Value("/"+ name + "/face/remoteControlBoard")},
                                         {"remote", yarp::os::Value("/" + robot + "/face")},
                                         {"part", yarp::os::Value("face")}};

        if (m_poly.open(rcb_face_conf))
        {
            yarp::dev::IControlMode* iCM{nullptr};
            yarp::dev::IControlLimits* iCtrlLim{nullptr};
            bool ok = m_poly.view(iCM);
            ok &= m_poly.view(m_iPos);
            ok &= m_poly.view(iCtrlLim);
            if (iCM)
                ok &= iCM->setControlMode(0, VOCAB_CM_POSITION);
            if (iCtrlLim)
            {
                ok &= iCtrlLim->getLimits(0, &m_minEyeLid, &m_maxEyeLid);
            }
            if (m_iPos)
            {
                ok &= m_iPos->setRefSpeed(0, 50.0); // max velocity that doesn't give problems
                ok &= m_iPos->setRefAcceleration(0, std::numeric_limits<double>::max());
            }
            if (!ok)
            {
                yError() << "Fail to configure correctly the remote_controlboard";
                return false;
            }
        }

        yInfo() << "Using Eye tracking.";
    }

    if (m_useLip)
    {
        output = ViveSR::anipal::Initial(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP, NULL);
        if (output != ViveSR::Error::WORK) {
            yError("[SRanipalModule::configure] Failed to initialize Lip engine [%d - %s].", output, errorCodeToString(output));
            return false;
        }

        std::string lipImageOutputPort = rf.check("lipImagePortName", yarp::os::Value("/lipImage:o"), "The name of the output port for the lip camera.").asString();
        if (!m_lipImagePort.open("/" + name + lipImageOutputPort))
        {
            yError() << "[SRanipalModule::configure] Failed to open /" + name + lipImageOutputPort + " port.";
            return false;
        }

        yInfo() << "Using Lip tracking.";
    }

    return true;
}

double SRanipalModule::getPeriod()
{
    return m_period;
}

bool SRanipalModule::updateModule()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    ViveSR::anipal::Eye::EyeData_v2 eye_data_v2;
    ViveSR::anipal::Lip::LipData lip_data_v2;
    lip_data_v2.image = m_lipImage;

    int result = ViveSR::Error::WORK;
    if (m_useEye) {
        int result = ViveSR::anipal::Eye::GetEyeData_v2(&eye_data_v2);
        if (result == ViveSR::Error::WORK) {
            std::string leftEyeBrow = "neu";
            std::string rightEyeBrow = "neu";
    
            if ((eye_data_v2.expression_data.left.eye_wide > m_eyeWideSurprisedThreshold) ||
                (eye_data_v2.expression_data.right.eye_wide > m_eyeWideSurprisedThreshold))
            {
                leftEyeBrow = "sur";
                rightEyeBrow = "sur";
            } 

            sendFaceExpression("leb", leftEyeBrow);
            sendFaceExpression("reb", rightEyeBrow);

        }
    }

    if (m_useLip) {
        result = ViveSR::anipal::Lip::GetLipData(&lip_data_v2);
        if (result == ViveSR::Error::WORK) {
            std::string mouthExpression = "neu";
            using namespace ViveSR::anipal::Lip;
            if ((lip_data_v2.prediction_data.blend_shape_weight[Jaw_Open] > m_lipExpressionThreshold) ||
                (lip_data_v2.prediction_data.blend_shape_weight[Mouth_O_Shape] > m_lipExpressionThreshold))
            {
                mouthExpression = "sur";
            }
            else if ((lip_data_v2.prediction_data.blend_shape_weight[Mouth_Smile_Left] > m_lipExpressionThreshold) ||
                     (lip_data_v2.prediction_data.blend_shape_weight[Mouth_Smile_Right] > m_lipExpressionThreshold))
            {
                mouthExpression = "hap";
            }
            else if ((lip_data_v2.prediction_data.blend_shape_weight[Mouth_Sad_Left] > m_lipExpressionThreshold) ||
                     (lip_data_v2.prediction_data.blend_shape_weight[Mouth_Sad_Right] > m_lipExpressionThreshold) ||
                     (lip_data_v2.prediction_data.blend_shape_weight[Mouth_Pout] > m_lipExpressionThreshold) || 
                     (lip_data_v2.prediction_data.blend_shape_weight[Mouth_Ape_Shape] > m_lipExpressionThreshold))
            {
                mouthExpression = "sad";
            }

            sendFaceExpression("mou", mouthExpression);

            yarp::sig::FlexImage& outputImage = m_lipImagePort.prepare();
            outputImage.setPixelCode(VOCAB_PIXEL_MONO);
            outputImage.setExternal(lip_data_v2.image, 800, 400);
            m_lipImagePort.write();
        }
    }

    return true;
}

bool SRanipalModule::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ViveSR::anipal::Release(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE_V2);
    ViveSR::anipal::Release(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP);
    m_emotionsOutputPort.close();
    m_poly.close();
    m_lipImagePort.close();
    m_iPos = nullptr;
    yInfo() << "Closing";
    return true;
}

