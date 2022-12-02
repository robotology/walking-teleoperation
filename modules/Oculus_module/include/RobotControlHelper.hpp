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
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

/**
 * RobotControlHelper is an helper class for controlling the robot.
 */
class RobotControlHelper
{
    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */

    int m_actuatedDOFs; /**< Number of the actuated DoF */

    std::vector<std::string>
        m_axesList; /**< Vector containing the name of the controlled joints. */

    yarp::dev::IPreciselyTimed* m_timedInterface{nullptr};
    yarp::dev::IEncodersTimed* m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* m_positionDirectInterface{nullptr}; /**< Direct position control
                                                                       interface. */
    yarp::dev::IPositionControl* m_positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl* m_velocityInterface{nullptr}; /**< Velocity control interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits* m_limitsInterface{nullptr}; /**< Encorders interface. */

    yarp::sig::Vector m_desiredJointValue; /**< Desired joint value [deg or deg/s]. */
    yarp::sig::Vector m_positionFeedbackInDegrees; /**< Joint position [deg]. */
    yarp::sig::Vector m_positionFeedbackInRadians; /**< Joint position [rad]. */
    yarp::os::Stamp m_timeStamp; /**< Time stamp. */

    bool m_isMandatory; /**< If false neglect the errors coming from the robot driver. */

    yarp::conf::vocab32_t m_controlMode; /**< Used control mode. */

    /**
     * Switch to control mode
     * @param controlMode is the specific control mode
     * @return true / false in case of success / failure
     */
    bool switchToControlMode(const int& controlMode);

    /**
     * Set the desired joint position (position direct mode)
     * desiredPosition desired joint position in radiant
     * @return true / false in case of success / failure
     */
    bool setDirectPositionReferences(const yarp::sig::Vector& desiredPositions);

    /**
     * Set the desired joint position (min jerk trajectory mode)
     * desiredPosition desired joint position in radiant
     * @return true / false in case of success / failure
     */
    bool setPositionReferences(const yarp::sig::Vector& desiredPositions);

    /**
     * Set the desired joint velocity
     * @param desiredVelocity desired joint velocity in radiant/s
     * @return true / false in case of success / failure
     */
    bool setVelocityReferences(const yarp::sig::Vector& desiredVelocity);

public:
    /**
     * Configure the helper
     * @param config confifuration options
     * @param name name of the robot
     * @param isMandatory if true the helper will return an error if there is a
     * problem in the configuration phase
     * @return true / false in case of success / failure
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name, bool isMandatory);

    /**
     * Update the time stamp
     */
    void updateTimeStamp();

    /**
     * Set the desired joint reference (position or velocity)
     * @param desiredValue desired joint velocity or position (radiant or radiant/s)
     * @return true / false in case of success / failure
     */
    bool setJointReference(const yarp::sig::Vector& desiredValue);

    /**
     * Check if the velocity control is used
     * @return true if the velocity control is used
     */
    bool isVelocityControlUsed();

    /**
     * Get feedback from the robot
     * @return true / false in case of success / failure
     */
    bool getFeedback();

    /**
     * Get the joint limits
     * @param limits matrix containing the joint limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(yarp::sig::Matrix& limits);

    /**
     * Get the time stamp (const version)
     * @return time stamp
     */
    const yarp::os::Stamp& timeStamp() const;

    /**
     * Get the time stamp
     * @return time stamp
     */
    yarp::os::Stamp& timeStamp();

    /**
     * Get the joint encoders value
     * @return the joint encoder value
     */
    const yarp::sig::Vector& jointEncoders() const;

    /**
     * Get the number of degree of freedom
     * @return the number of actuated DoF
     */
    int getDoFs();

    /**
     * Close the helper
     */
    void close();
};

#endif
