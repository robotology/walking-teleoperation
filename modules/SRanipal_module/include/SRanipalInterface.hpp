// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#ifndef SRANIPALINTERFACE_HPP
#define SRANIPALINTERFACE_HPP

#include <SRanipal.h>
#include <SRanipal_Eye.h>
#include <SRanipal_Lip.h>
#include <SRanipal_Enums.h>
#include <SRanipal_NotRelease.h>
#include <iDynTree/Core/Axis.h>

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

    bool getEyeOpenness(double &left_openness, double &right_openness);

    bool getEyeWideness(double& wideness);

    bool getGazeAxes(iDynTree::Axis& leftEyeGaze, iDynTree::Axis& rightEyeGaze);

    bool getLipExpressions(LipExpressions& expressions);

    const char* lipImage();

    void close();
};

#endif // SRANIPALINTERFACE_HPP
