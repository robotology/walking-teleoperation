// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef VRINTERFACE_HPP
#define VRINTERFACE_HPP

#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <iDynTree/Core/Transform.h>


class VRInterface
{
    struct EyeControl
    {
        yarp::os::BufferedPort<yarp::sig::Vector> imageControlPort;
        double elevation;
        double azimuth;
        iDynTree::Position eyePosition;
        iDynTree::Position imageRelativePosition;

        void sendAngles();

        iDynTree::Transform currentImageTransform(); //With respect to the headset frame

        bool intersectionInImage(const iDynTree::Axis& operatorGazeInSRanipalFrame, iDynTree::Vector2 &output);

        void close();
    };

    std::string m_name;
    bool m_isActive = false;
    bool m_deadzoneActive = false;
    double m_deadzoneActivationTime = -1.0;
    double m_deadzoneMinActivationTimeInS = 0.5;
    double m_lastActiveCheck{-1.0};
    double m_velocityGain{0.0};
    double m_errorDeadzone{0.01};
    double m_errorDeadzoneActivation{0.1};
    double m_eyeMovementAccuracyInRad{0.0001};
    yarp::os::RpcClient m_VRDeviceRPCOutputPort;
    EyeControl m_leftEye, m_rightEye;

    bool getValueFromRPC(const std::string& query, yarp::os::Value& value);

    bool getValueFromRPC(const std::string& query, bool& value);

    bool getValueFromRPC(const std::string& query, double& value);

    bool getValueFromRPC(const std::string& query, std::string& value);

    iDynTree::Vector2 applyDeadzone(const iDynTree::Vector2& input);

    double applyQuantization(double input, double quantization);

public:

    bool configure(const yarp::os::ResourceFinder& rf);

    void setVRImagesPose(double vergenceInRad, double versionInRad, double tiltInRad);

    bool computeDesiredRobotEyeVelocities(const iDynTree::Axis& operatorLeftEyeGaze, const iDynTree::Axis& operatorRightEyeGaze,
                                          double& vergenceSpeedInRadS, double& versionSpeedInRadS, double& tiltSpeedInRadS);

    bool setGUIEnabled(int index, bool enabled);

    bool isActive();

    void close();
};

#endif // VRINTERFACE_HPP
