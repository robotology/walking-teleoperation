// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <SRanipalInterface.hpp>
#include <yarp/os/LogStream.h>
#include <cmath>
#include <algorithm>


const char *SRanipalInterface::errorCodeToString(int error) const
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

SRanipalInterface::SRanipalInterface()
{
    m_lipData.image = m_lipImage;
}

SRanipalInterface::~SRanipalInterface()
{
    close();
}

bool SRanipalInterface::initializeEyeEngine()
{
    if (m_eyeInitialized)
    {
        yError() << "[SRanipalInterface::initializeEyeEngine] The Eye engine is already initialize.";
        return false;
    }

    int output = ViveSR::anipal::Initial(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE_V2, NULL);
    if (output != ViveSR::Error::WORK) {
        if (output == ViveSR::Error::RUNTIME_NOT_FOUND)
        {
            yError() << "[SRanipalInterface::initializeEyeEngine] SRanipal runtime non installed/available.";
        }
        else if (output == ViveSR::Error::NOT_SUPPORT_EYE_TRACKING)
        {
            yError() << "[SRanipalInterface::initializeEyeEngine] This HMD do not have eye tracking feature!";
        }
        yError("[SRanipalInterface::initializeEyeEngine] Failed to initialize Eye engine [%d - %s].", output, errorCodeToString(output));
        return false;
    }

    m_eyeInitialized = true;

    return true;
}

bool SRanipalInterface::isEyeCalibrationNeeded(bool& calibrationIsNeeded)
{
    if (!m_eyeInitialized)
    {
        yError() << "[SRanipalInterface::initializeEyeEngine] The Eye engine is not initialized.";
        calibrationIsNeeded = true;
        return false;
    }

    bool needCalibration = false;
    int output = ViveSR::anipal::Eye::IsUserNeedCalibration(&needCalibration);
    if (output != ViveSR::Error::WORK) {
        yError("[SRanipalInterface::initializeEyeEngine] Failed to initialize check if the eye calibration is needed [%d - %s].", output, errorCodeToString(output));
        calibrationIsNeeded = true;
        return false;
    }

    calibrationIsNeeded = needCalibration;

    return true;
}

bool SRanipalInterface::calibrateEyeTracking()
{
    if (!m_eyeInitialized)
    {
        yError() << "[SRanipalInterface::calibrateEyeTracking] The Eye engine is not initialized.";
        return false; //In this way we actually suggest the user to call the calibration procedure, and fail also there
    }

    int output = ViveSR::anipal::Eye::LaunchEyeCalibration(nullptr);
    if (output != ViveSR::Error::WORK) {
        yError("[SRanipalModule::configure] Failed to calibrate the eyes [%d - %s].", output, errorCodeToString(output));
        return false;
    }

    return true;
}

bool SRanipalInterface::initializeLipEngine()
{
    if (m_lipInitialized)
    {
        yError() << "[SRanipalInterface::initializeEyeEngine] The Lip engine is already initialize.";
        return false;
    }

    int output = ViveSR::anipal::Initial(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP, NULL); //LIP V1 works better in detecting the smile
    if (output != ViveSR::Error::WORK) {
        yError("[SRanipalModule::configure] Failed to initialize Lip engine [%d - %s].", output, errorCodeToString(output));
        return false;
    }

    m_lipInitialized = true;

    return true;
}

bool SRanipalInterface::updateEyeData()
{
    bool okEye{false};

    if (m_eyeInitialized)
    {
        okEye = ViveSR::anipal::Eye::GetEyeData_v2(&m_eyeData_v2) == ViveSR::Error::WORK;
    }

    m_eyeUpdated = okEye;

    return okEye;
}

bool SRanipalInterface::updateLipData()
{
    bool okLip{false};

    if (m_lipInitialized)
    {
        okLip = ViveSR::anipal::Lip::GetLipData(&m_lipData) == ViveSR::Error::WORK;
    }

    m_lipUpdated = okLip;

    return okLip;
}

bool SRanipalInterface::getEyeOpenness(double &left_openness, double &right_openness)
{
    using namespace ViveSR::anipal::Eye;
    bool eye_openness_validity = m_eyeUpdated &&
            DecodeBitMask(m_eyeData_v2.verbose_data.left.eye_data_validata_bit_mask,
                          SingleEyeDataValidity::SINGLE_EYE_DATA_EYE_OPENNESS_VALIDITY) &&
            DecodeBitMask(m_eyeData_v2.verbose_data.right.eye_data_validata_bit_mask,
                          SingleEyeDataValidity::SINGLE_EYE_DATA_EYE_OPENNESS_VALIDITY) &&
            m_eyeData_v2.no_user; //no user is false if the headset is removed

    left_openness = 1.0;
    right_openness = 1.0;

    if (eye_openness_validity)
    {
        left_openness = m_eyeData_v2.verbose_data.left.eye_openness;
        right_openness = m_eyeData_v2.verbose_data.right.eye_openness;
    }

    return eye_openness_validity;
}

bool SRanipalInterface::getEyeWideness(double &wideness)
{
    wideness = 0.0;
    if (!m_eyeUpdated)
    {
        return false;
    }

    wideness = std::max(m_eyeData_v2.expression_data.left.eye_wide, m_eyeData_v2.expression_data.right.eye_wide);

    return true;
}

bool SRanipalInterface::getGazeAxes(iDynTree::Axis &leftEyeGaze, iDynTree::Axis &rightEyeGaze)
{
    using namespace ViveSR::anipal::Eye;
    bool eye_gaze_validity = m_eyeUpdated &&
            DecodeBitMask(m_eyeData_v2.verbose_data.left.eye_data_validata_bit_mask,
                          SingleEyeDataValidity::SINGLE_EYE_DATA_GAZE_DIRECTION_VALIDITY) &&
            DecodeBitMask(m_eyeData_v2.verbose_data.left.eye_data_validata_bit_mask,
                          SingleEyeDataValidity::SINGLE_EYE_DATA_GAZE_ORIGIN_VALIDITY) &&
            DecodeBitMask(m_eyeData_v2.verbose_data.right.eye_data_validata_bit_mask,
                          SingleEyeDataValidity::SINGLE_EYE_DATA_GAZE_DIRECTION_VALIDITY) &&
            DecodeBitMask(m_eyeData_v2.verbose_data.right.eye_data_validata_bit_mask,
                          SingleEyeDataValidity::SINGLE_EYE_DATA_GAZE_ORIGIN_VALIDITY) &&
            m_eyeData_v2.no_user; //no user is false if the headset is removed

    if (!eye_gaze_validity)
    {
        return false;
    }

    iDynTree::Position origin;
    iDynTree::Direction direction;

    origin[0] = m_eyeData_v2.verbose_data.left.gaze_origin_mm.x/1000.0;
    origin[1] = m_eyeData_v2.verbose_data.left.gaze_origin_mm.y/1000.0;
    origin[2] = m_eyeData_v2.verbose_data.left.gaze_origin_mm.z/1000.0;
    leftEyeGaze.setOrigin(origin);


    direction[0] = m_eyeData_v2.verbose_data.left.gaze_direction_normalized.x;
    direction[1] = m_eyeData_v2.verbose_data.left.gaze_direction_normalized.y;
    direction[2] = m_eyeData_v2.verbose_data.left.gaze_direction_normalized.z;
    leftEyeGaze.setDirection(direction);

    origin[0] = m_eyeData_v2.verbose_data.right.gaze_origin_mm.x/1000.0;
    origin[1] = m_eyeData_v2.verbose_data.right.gaze_origin_mm.y/1000.0;
    origin[2] = m_eyeData_v2.verbose_data.right.gaze_origin_mm.z/1000.0;
    rightEyeGaze.setOrigin(origin);


    direction[0] = m_eyeData_v2.verbose_data.right.gaze_direction_normalized.x;
    direction[1] = m_eyeData_v2.verbose_data.right.gaze_direction_normalized.y;
    direction[2] = m_eyeData_v2.verbose_data.right.gaze_direction_normalized.z;
    rightEyeGaze.setDirection(direction);

    return true;
}

bool SRanipalInterface::getLipExpressions(LipExpressions &expressions)
{
    using namespace ViveSR::anipal::Lip;

    if (!m_lipUpdated)
    {
        expressions = LipExpressions();
        return false;
    }

    expressions.mouthOpen = std::max(m_lipData.prediction_data.blend_shape_weight[Jaw_Open],
                                     m_lipData.prediction_data.blend_shape_weight[Mouth_O_Shape]);

    expressions.smile = std::max(m_lipData.prediction_data.blend_shape_weight[Mouth_Smile_Left],
                                 m_lipData.prediction_data.blend_shape_weight[Mouth_Smile_Right]);

    expressions.sad = std::max({m_lipData.prediction_data.blend_shape_weight[Mouth_Sad_Left],
                                m_lipData.prediction_data.blend_shape_weight[Mouth_Sad_Right],
                                m_lipData.prediction_data.blend_shape_weight[Mouth_Pout],
                                m_lipData.prediction_data.blend_shape_weight[Mouth_Ape_Shape]});

    return true;
}

const char *SRanipalInterface::lipImage()
{
    return m_lipImage;
}

void SRanipalInterface::close()
{
    if (m_eyeInitialized)
    {
        ViveSR::anipal::Release(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE_V2);
        m_eyeInitialized = false;
    }

    if (m_lipInitialized)
    {
        ViveSR::anipal::Release(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP);
        m_lipInitialized = false;
    }
}
