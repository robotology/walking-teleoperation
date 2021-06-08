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

bool SRanipalModule::configure(yarp::os::ResourceFinder &rf)
{
    int output;

    std::string name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();
    setName(name.c_str());

    std::string emotionsPortOut = rf.check("emotions_output_port_name", yarp::os::Value("/emotions:o"), "The name of the output port for the emotions.").asString();
    m_emotionsOutputPort.open("/" + name + emotionsPortOut);

    m_useEye = !rf.check("noEye") || (!rf.find("noEye").asBool()); //True if noEye is not set or set to false
    m_useLip = !rf.check("noLip") || (!rf.find("noLip").asBool()); //True if noLip is not set or set to false
    m_period = rf.check("period", yarp::os::Value(0.01)).asDouble();

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
    }

    if (m_useLip)
    {
        output = ViveSR::anipal::Initial(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP_V2, NULL);
        if (output != ViveSR::Error::WORK) {
            yError("[SRanipalModule::configure] Failed to initialize Lip engine [%d - %s].", output, errorCodeToString(output));
            return false;
        }
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
    ViveSR::anipal::Lip::LipData_v2 lip_data_v2;
    lip_data_v2.image = m_lipImage;

    int result = ViveSR::Error::WORK;
    if (m_useEye) {
        int result = ViveSR::anipal::Eye::GetEyeData_v2(&eye_data_v2);
        if (result == ViveSR::Error::WORK) {
            //TODO
        }
    }
    if (m_useLip) {
        result = ViveSR::anipal::Lip::GetLipData_v2(&lip_data_v2);
        if (result == ViveSR::Error::WORK) {
            //TODO
        }
    }

    return true;
}

bool SRanipalModule::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ViveSR::anipal::Release(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE_V2);
    ViveSR::anipal::Release(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP_V2);
    m_emotionsOutputPort.close();
    yInfo() << "Closing";
    return true;
}

