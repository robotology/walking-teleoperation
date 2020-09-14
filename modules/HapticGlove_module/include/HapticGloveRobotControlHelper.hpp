/**
 * @file HapticGloveRobotControlHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef HAPTIC_GLOVE_ROBOT_CONTROL_HELPER_HPP
#define HAPTIC_GLOVE_ROBOT_CONTROL_HELPER_HPP

// std
#include <memory>

// YARP
#include <yarp/dev/IAnalogSensor.h>
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

/**
 * RobotControlHelper is an helper class for controlling the robot.
 */
namespace HapticGlove
{

class RobotControlHelper
{
    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    yarp::dev::PolyDriver m_analogDevice; /**< Analog device. */

    int m_actuatedDOFs; /**< Number of the actuated DoF */
    size_t m_noAnalogSensor; /**< Number of the joints ( associated with the analog sensors) */

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
    yarp::dev::IAnalogSensor* m_AnalogSensorInterface{nullptr}; /*Sensor interface*< */

    yarp::sig::Vector m_desiredJointValue; /**< Desired joint value [deg or deg/s]. */
    yarp::sig::Vector m_positionFeedbackInDegrees; /**< Joint position [deg]. */
    yarp::sig::Vector m_positionFeedbackInRadians; /**< Joint position [rad]. */
    yarp::sig::Vector m_sensorFeedbackRaw; /**< sensor feedback [raw]*/
    yarp::sig::Vector m_sensorFeedbackInDegrees; /**< sensor feedback [deg]*/
    yarp::sig::Vector m_sensorFeedbackInRadians; /**< sensor feedback [rad]*/
    yarp::sig::Vector m_sensorFeedbackSelected; /**< sensor info to read*/

    yarp::sig::Vector m_joints_min_boundary; /**< joint minimum possible value [deg]*/
    yarp::sig::Vector m_joints_max_boundary; /**< joint maximum possible value [deg]*/
    yarp::sig::Vector m_sensors_min_boundary; /**< senor minimum value [raw]*/
    yarp::sig::Vector m_sensors_max_boundary; /**< senor maximum value [raw]*/
    yarp::sig::Vector m_sensors_raw2Degree_scaling; /**< sacling from raw to Degree of joints*/


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
     * Get calibrated sensory feedback from the robot
     * @return true / false in case of success / failure
     */
    bool getCalibratedFeedback();

    /**
     * Get the joint limits
     * @param limits matrix containing the joint limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(yarp::sig::Matrix& limits);

    /**
     * Get the joint velocity limits
     * @param limits matrix containing the joint velocity limits in radian/sec
     * @return true / false in case of success / failure
     */
    bool getVelocityLimits(yarp::sig::Matrix& limits);

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
     * Get the analog sensors value
     * @return the analog sensor values
     */
    const yarp::sig::Vector& analogSensors() const;

    /**
     * Get the number of actuated degree of freedom (motors)
     * @return the number of actuated DoF
     */
    int getActuatedDoFs();

    /**
     * Get the number of joints
     * @return the number of joints
     */
    int getNumberOfJoints();

    /**
     * Close the helper
     */
    void close();
};
} // namespace HapticGlove

#endif
