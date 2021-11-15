/**
 * @file GazeRetargeting.hpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef GAZERETARGETING_HPP
#define GAZERETARGETING_HPP

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>

class GazeRetargeting
{
    class VRInterface
    {
        struct EyeControl
        {
            yarp::os::BufferedPort<yarp::sig::Vector> controlPort;
            double elevation;
            double azimuth;
            iDynTree::Position eyePosition;
            iDynTree::Position imageRelativePosition;

            void sendAngles();

            iDynTree::Transform currentImageTransform(); //With respect to the headset frame

            bool intersectionInImage(const iDynTree::Axis& gazeInSRanipalFrame, iDynTree::Vector2 &output);

            void close();
        };

        std::string m_name;
        bool m_isActive = false;
        double m_lastActiveCheck{-1.0};
        double m_velocityGain{0.0};
        double m_errorDeadzone{0.01};
        yarp::os::RpcClient m_VRDeviceRPCOutputPort;
        EyeControl m_leftEye, m_rightEye;

        bool getValueFromRPC(const std::string& query, yarp::os::Value& value);

        bool getValueFromRPC(const std::string& query, bool& value);

        bool getValueFromRPC(const std::string& query, double& value);

        bool getValueFromRPC(const std::string& query, std::string& value);

        iDynTree::Vector2 applyDeadzone(const iDynTree::Vector2& input);

    public:

        bool configure(yarp::os::ResourceFinder& rf);

        void setVRImagesPose(double vergenceInRad, double versionInRad, double tiltInRad);

        bool computeDesiredEyeVelocities(const iDynTree::Axis& leftEyeGaze, const iDynTree::Axis& rightEyeGaze,
                                         double& vergenceSpeedInRadS, double& versionSpeedInRadS, double& tiltSpeedInRadS);

        bool isActive();

        void close();
    };


    bool m_configured{false};
    yarp::dev::PolyDriver m_eyesDriver;
    yarp::dev::IVelocityControl* m_eyesVel{nullptr};
    yarp::dev::IPositionControl* m_eyesPos{nullptr};
    yarp::dev::IEncoders* m_eyesEnc{nullptr};
    yarp::dev::IControlMode* m_eyesMode{nullptr};
    int m_eyeVersIndex, m_eyeVergIndex, m_eyeTiltIndex;
    double m_eyeVersInRad, m_eyeVergInRad, m_eyeTiltInRad;
    double m_maxTiltInDeg, m_maxVersInDeg, m_maxVergInDeg;
    double m_tanhGain;
    std::vector<double> m_encodersInDeg;
    double m_maxEyeSpeedInDegS;
    iDynTree::Axis m_leftGaze, m_rightGaze;
    bool m_gazeSet{false};

    VRInterface m_VRInterface;

    void setEyeControlMode(int controlMode);

    bool homeEyes();

    bool updateEyeEncoders();

    bool setDesiredEyeVelocities(double vergenceSpeedInDeg, double versionSpeedInDeg, double tiltSpeedInDeg);

    double saturateEyeVelocity(double inputVelocity, double inputPosition,
                               double maxVelocity, double kinematicLowerBound, double kinematicUpperBound);

public:

    GazeRetargeting() = default;

    GazeRetargeting(const GazeRetargeting& other) = delete;

    GazeRetargeting(GazeRetargeting&& other) = delete;

    ~GazeRetargeting();

    GazeRetargeting& operator=(const GazeRetargeting& other) = delete;

    GazeRetargeting& operator=(GazeRetargeting&& other) = delete;

    bool configure(yarp::os::ResourceFinder& rf);

    void setEyeGazeAxes(const iDynTree::Axis& leftGaze, const iDynTree::Axis& rightGaze);

    bool update();

    void close();
};
#endif // GAZERETARGETING_HPP
