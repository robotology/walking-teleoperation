/**
 * @file SRanipalInterface.hpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */


#ifndef SRANIPALINTERFACE_HPP
#define SRANIPALINTERFACE_HPP

#include <SRanipal.h>
#include <SRanipal_Eye.h>
#include <SRanipal_Lip.h>
#include <SRanipal_Enums.h>
#include <SRanipal_NotRelease.h>

class SRanipalInterface
{
    bool m_lipInitialized{false};
    bool m_eyeInitialized{false};

    bool m_lipUpdated{false};
    bool m_eyeUpdated{false};

    ViveSR::anipal::Eye::EyeData_v2 m_eyeData_v2;
    ViveSR::anipal::Lip::LipData m_lipData; //The lip data V1 works better in detecting the smile
    char m_lipImage[800 * 400];

    const char * errorCodeToString(int error) const;


public:

    struct LipExpressions
    {
        double mouthOpen{0.0};
        double smile{0.0};
        double sad{0.0};
    };

    SRanipalInterface();

    SRanipalInterface(const SRanipalInterface& other) = delete;

    SRanipalInterface(SRanipalInterface&& other) = delete;

    SRanipalInterface& operator=(const SRanipalInterface& other) = delete;

    SRanipalInterface& operator=(SRanipalInterface&& other) = delete;

    ~SRanipalInterface();

    bool initializeEyeEngine();

    bool isEyeCalibrationNeeded(bool &calibrationIsNeeded);

    bool calibrateEyeTracking();

    bool initializeLipEngine();

    bool updateEyeData();

    bool updateLipData();

    bool getEyeOpenness(double& openness);

    bool getEyeWideness(double& wideness);

    bool getLipExpressions(LipExpressions& expressions);

    const char* lipImage();

    void close();
};

#endif // SRANIPALINTERFACE_HPP
