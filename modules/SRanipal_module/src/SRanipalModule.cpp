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
#include <algorithm>
#include <thread>
#include <yarp/os/LogStream.h>
#include <yarp/dev/IControlLimits.h>
#include <SRanipal.h>
#include <SRanipal_Eye.h>
#include <SRanipal_Lip.h>
#include <SRanipal_Enums.h>
#include <SRanipal_NotRelease.h>
#include <cmath>

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
        cmd.addVocab32(yarp::os::Vocab32::encode("set"));
        cmd.addVocab32(yarp::os::Vocab32::encode(part));
        cmd.addVocab32(yarp::os::Vocab32::encode(emotion));
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
    m_useEyelids = !rf.check("noEyelids") || (!rf.find("noEyelids").asBool()); //True if noEyelids is not set or set to false
    m_useRawEyelids = rf.check("useRawEyelids")
            && (rf.find("useRawEyelids").isNull() || rf.find("useRawEyelids").asBool());
    m_useEyelidsPositionControl = rf.check("useEyelidsPositionControl")
            && (rf.find("useEyelidsPositionControl").isNull() || rf.find("useEyelidsPositionControl").asBool());
    m_eyelidsMaxVelocity = rf.check("eyelidsMaxVelocity", yarp::os::Value(75.0)).asFloat64();
    m_eyelidsVelocityGain = rf.check("eyelidsVelocityGain", yarp::os::Value(10.0)).asFloat64();
    bool eyelidsVelocityControl = m_useEyelids && !(m_useRawEyelids || m_useEyelidsPositionControl);
    double defaultPeriod = eyelidsVelocityControl ? 0.01 : 0.1; //If we control the eyelids in velocity control, we use higher loops.
    m_period = rf.check("period", yarp::os::Value(defaultPeriod)).asFloat64();
    m_lipExpressionThreshold = rf.check("lipExpressionThreshold", yarp::os::Value(0.2)).asFloat64();
    m_eyeWideSurprisedThreshold = rf.check("eyeWideSurprisedThreshold", yarp::os::Value(0.2)).asFloat64();
    m_eyeOpenPrecision = rf.check("eyeOpenPrecision", yarp::os::Value(0.1)).asFloat64();

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

        if (m_useEyelids)
        {
            m_rawEyelidsCloseValue = rf.check("rawEyelidsCloseValue", yarp::os::Value(35)).asInt32(); //The default value has been found on the greeny
            m_rawEyelidsOpenValue  = rf.check("rawEyelidsOpenValue",  yarp::os::Value(60)).asInt32(); // The default value has been found on the greeny
            std::string rawEyelidsPortName = rf.check("rawEyelidsPortName", yarp::os::Value("/face/raw:o")).asString();
            if (m_useRawEyelids)
            {
                if (!m_rawEyelidsOutputPort.open("/" + name + rawEyelidsPortName))
                {
                    yError() << "[SRanipalModule::configure] Failed to open /" + name
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
                        yError() << "Failed to configure the eyelids remote_controlboard.";
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
                            yError() << "Failed to configure the eyelids control.";
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
                        yError() << "Failed to get the eyelids limits.";
                        return false;
                    }

                }
                else
                {
                    yError() << "Failed to connect to the face control board. Set noEyelids to "
                                "true to avoid connecting to it.";
                    return false;
                }
            }

            yInfo() << "Controlling the eyelids.";
        }
    }

    if (m_useLip)
    {
        output = ViveSR::anipal::Initial(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP, NULL); //LIP V1 works better in detecting the smile
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

    ViveSR::anipal::Eye::EyeData_v2 eye_data_v2;
    ViveSR::anipal::Lip::LipData lip_data; //The lip data V1 works better in detecting the smile
    lip_data.image = m_lipImage;

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

            if (m_useEyelids)
            {
                bool eye_openness_validity = ViveSR::anipal::Eye::DecodeBitMask(eye_data_v2.verbose_data.left.eye_data_validata_bit_mask,
                                                                                ViveSR::anipal::Eye::SingleEyeDataValidity::SINGLE_EYE_DATA_EYE_OPENNESS_VALIDITY) &&
                        ViveSR::anipal::Eye::DecodeBitMask(eye_data_v2.verbose_data.right.eye_data_validata_bit_mask,
                                                           ViveSR::anipal::Eye::SingleEyeDataValidity::SINGLE_EYE_DATA_EYE_OPENNESS_VALIDITY) &&
                        !eye_data_v2.no_user;

                if (eye_openness_validity)
                {
                    double eye_openness = std::min(eye_data_v2.verbose_data.left.eye_openness, eye_data_v2.verbose_data.right.eye_openness);
                    int eye_open_level = static_cast<int>(std::round(eye_openness / m_eyeOpenPrecision));
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
                            yInfo() << "Setting eye openess (raw):" << eye_openess_leveled;
                            yDebug() << "Sending raw commands to eyelids:" << out.toString();
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
                                yInfo() << "Setting eye openess (position):" << eye_openess_leveled;
                            }
                        }
                        else
                        {
                            if (m_eyelidsVel && m_eyelidsEnc)
                            {
                                if (eye_open_level != m_eyeOpenLevel)
                                {
                                    m_eyeOpenLevel = eye_open_level;
                                    yInfo() << "Setting eye openess (velocity):" << eye_openess_leveled;
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
                                    yWarning() << "Failed to get eyelids encoder value. Skipping control.";
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (m_useLip) {
        result = ViveSR::anipal::Lip::GetLipData(&lip_data);
        if (result == ViveSR::Error::WORK) {
            std::string mouthExpression = "neu";
            using namespace ViveSR::anipal::Lip;
            if ((lip_data.prediction_data.blend_shape_weight[Jaw_Open] > m_lipExpressionThreshold) ||
                (lip_data.prediction_data.blend_shape_weight[Mouth_O_Shape] > m_lipExpressionThreshold))
            {
                mouthExpression = "sur";
            }
            else if ((lip_data.prediction_data.blend_shape_weight[Mouth_Smile_Left] > m_lipExpressionThreshold) ||
                     (lip_data.prediction_data.blend_shape_weight[Mouth_Smile_Right] > m_lipExpressionThreshold))
            {
                mouthExpression = "hap";
            }
            else if ((lip_data.prediction_data.blend_shape_weight[Mouth_Sad_Left] > m_lipExpressionThreshold) ||
                     (lip_data.prediction_data.blend_shape_weight[Mouth_Sad_Right] > m_lipExpressionThreshold) ||
                     (lip_data.prediction_data.blend_shape_weight[Mouth_Pout] > m_lipExpressionThreshold) ||
                     (lip_data.prediction_data.blend_shape_weight[Mouth_Ape_Shape] > m_lipExpressionThreshold))
            {
                mouthExpression = "sad";
            }

            sendFaceExpression("mou", mouthExpression);

            yarp::sig::FlexImage& outputImage = m_lipImagePort.prepare();
            outputImage.setPixelCode(VOCAB_PIXEL_MONO);
            outputImage.setExternal(lip_data.image, 800, 400);
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
    if (m_eyelidsMode)
    {
        m_eyelidsMode->setControlMode(0, VOCAB_CM_POSITION);
        m_eyelidsMode = nullptr;
    }
    m_eyelidsDriver.close();
    m_lipImagePort.close();
    m_rawEyelidsOutputPort.close();
    m_eyelidsPos = nullptr;
    yInfo() << "Closing";
    return true;
}

