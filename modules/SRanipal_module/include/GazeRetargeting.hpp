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
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <VRInterface.hpp>
#include <memory>
#include <vector>

class GazeRetargeting
{
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
    iDynTree::Axis m_leftGazeOperator, m_rightGazeOperator;
    std::vector<int> m_eyeAxis;
    std::vector<int> m_eyeControlModes;
    std::vector<double> m_eyemaxPositionMoveSpeeds;
    std::vector<double> m_eyePositionReferences;
    std::vector<double> m_eyesVelocityReferences;
    double* m_versionVelocityptr{nullptr};
    double* m_vergenceVelocityptr{nullptr};
    double* m_tiltVelocityptr{nullptr};
    double m_dummy{0.0};
    bool m_gazeSet{false};
    bool m_useVersion{true};
    bool m_useVergence{true};
    bool m_useTilt{true};

    std::shared_ptr<VRInterface> m_VRInterface;

    void setRobotEyeControlMode(int controlMode);

    bool homeRobotEyes();

    bool updateRobotEyeEncoders();

    bool setDesiredRobotEyeVelocities(double vergenceSpeedInDegS, double versionSpeedInDegS, double tiltSpeedInDegS);

    double saturateRobotEyeVelocity(double inputVelocity, double inputPosition,
                                    double maxVelocity, double jointLowerBound, double jointUpperBound);

public:

    GazeRetargeting() = default;

    GazeRetargeting(const GazeRetargeting& other) = delete;

    GazeRetargeting(GazeRetargeting&& other) = delete;

    ~GazeRetargeting();

    GazeRetargeting& operator=(const GazeRetargeting& other) = delete;

    GazeRetargeting& operator=(GazeRetargeting&& other) = delete;

    bool configure(const yarp::os::ResourceFinder& rf, std::shared_ptr<VRInterface> vrInterface);

    void setOperatorEyeGazeAxes(const iDynTree::Axis& leftGaze, const iDynTree::Axis& rightGaze);

    bool update();

    void close();
};
#endif // GAZERETARGETING_HPP
