/**
 * @file RobotControlHelper.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef ROBOT_CONTROL_HELPER_HPP
#define ROBOT_CONTROL_HELPER_HPP

// std
#include <memory>

// YARP
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

// iDynTree

class RobotControlHelper
{
    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */

    int m_actuatedDOFs; /**< Number of the actuated DoF */

    yarp::dev::IPreciselyTimed* m_timedInterface{nullptr};
    yarp::dev::IEncodersTimed* m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* m_positionDirectInterface{nullptr}; /**< Direct position control
                                                                       interface. */
    yarp::dev::IPositionControl* m_positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits* m_limitsInterface{nullptr}; /**< Encorders interface. */

    yarp::sig::Vector m_desiredPositionInDegrees; /**< Vector containing desired joint
                                                     position [deg]. */
    yarp::sig::Vector m_positionFeedbackInDegrees;
    yarp::sig::Vector m_positionFeedbackInRadians;
    yarp::sig::Vector m_minJointsPosition;
    yarp::sig::Vector m_maxJointsPosition;
    yarp::os::Stamp m_timeStamp;

    bool m_isActive;

    bool switchToControlMode(const int& controlMode);

public:
    bool configure(const yarp::os::Searchable& config, const std::string& name);

    bool setDirectPositionReferences(const yarp::sig::Vector& desiredPositions);

    void updateTimeStamp();

    bool getFeedback();

    const yarp::os::Stamp& timeStamp() const;

    yarp::os::Stamp& timeStamp();

    const yarp::sig::Vector& jointEncoders() const;

    void close();

    int getDoFs();
};

#endif
