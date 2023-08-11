// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef EYELIDSRETARGETING_HPP
#define EYELIDSRETARGETING_HPP

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>

class EyelidsRetargeting
{

    bool m_configured{false};
    bool m_useRawEyelids;
    bool m_useEyelidsPositionControl;
    double m_eyelidsPrecision;
    double m_eyelidsMaxVelocity;
    double m_eyelidsVelocityGain;
    int m_rawEyelidsCloseValue;
    int m_rawEyelidsOpenValue;
    int m_eyeClosenessLevel{-1};
    double m_desiredEyeOpenness{1.0};
    yarp::os::BufferedPort<yarp::os::Bottle> m_rawEyelidsOutputPort;
    yarp::dev::PolyDriver m_eyelidsDriver;
    yarp::dev::IPositionControl* m_eyelidsPos{nullptr};
    yarp::dev::IVelocityControl* m_eyelidsVel{nullptr};
    yarp::dev::IEncoders* m_eyelidsEnc{nullptr};
    yarp::dev::IControlMode* m_eyelidsMode{nullptr};
    double m_minEyeLid, m_maxEyeLid;

    bool rawRobotEyelidsControl(int eye_closeness_level);
    bool rfeRobotEyelidsControl(int eye_closeness_level);


public:

    EyelidsRetargeting() = default;

    EyelidsRetargeting(const EyelidsRetargeting& other) = delete;

    EyelidsRetargeting(EyelidsRetargeting&& other) = delete;

    ~EyelidsRetargeting();

    EyelidsRetargeting& operator=(const EyelidsRetargeting& other) = delete;

    EyelidsRetargeting& operator=(EyelidsRetargeting&& other) = delete;

    bool configure(yarp::os::ResourceFinder& rf);

    bool usingEylidsVelocityControl();

    void setDesiredEyeOpennes(double leftEyeOpennes, double rightEyeOpennes);

    bool update();

    void close();
};

#endif // EYELIDSRETARGETING_HPP
