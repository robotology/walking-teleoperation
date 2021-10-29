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

class GazeRetargeting
{
    class VRInterface
    {
        std::string m_name;
        bool m_isActive = false;
        double m_lastActiveCheck{-1.0};
        double m_IPD, m_eyesZPosition;
        yarp::os::RpcClient m_VRDeviceRPCOutputPort;
        yarp::os::BufferedPort<yarp::sig::Vector> m_leftEyeControlPort, m_rightEyeControlPort;

        bool getValueFromRPC(const std::string& query, yarp::os::Value& value);

        bool getValueFromRPC(const std::string& query, bool& value);

        bool getValueFromRPC(const std::string& query, double& value);

        bool getValueFromRPC(const std::string& query, std::string& value);

    public:

        bool configure(yarp::os::ResourceFinder& rf);

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

    VRInterface m_VRInterface;

    void setEyeControlMode(int controlMode);

public:

    GazeRetargeting() = default;

    GazeRetargeting(const GazeRetargeting& other) = delete;

    GazeRetargeting(GazeRetargeting&& other) = delete;

    ~GazeRetargeting();

    GazeRetargeting& operator=(const GazeRetargeting& other) = delete;

    GazeRetargeting& operator=(GazeRetargeting&& other) = delete;

    bool configure(yarp::os::ResourceFinder& rf);

    bool update();

    void close();
};
#endif // GAZERETARGETING_HPP
