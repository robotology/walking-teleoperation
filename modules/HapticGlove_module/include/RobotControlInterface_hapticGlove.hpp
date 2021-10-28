/**
 * @file HapticGloveRobotControlInterface.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef ROBOT_CONTROL_INTERFACE_HAPTIC_GLOVE_HPP
#define ROBOT_CONTROL_INTERFACE_HAPTIC_GLOVE_HPP

// std
#include <memory>

// YARP
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

namespace HapticGlove
{
class RobotControlInterface;
struct axisSensorData;
} // namespace HapticGlove

/**
 * Data structure the name of the all the sesnors (analog, encoders) assosiated with the axes list.
 */
struct HapticGlove::axisSensorData
{
    std::string axisName;
    bool useAnalog;
    std::vector<int> relatedAnalogSensorsIndex; /**< The list of elements of the analog sensor
                                                   associated axis name */
};

/**
 * RobotControlInterface is an helper class for controlling the robot.
 */
class HapticGlove::RobotControlInterface
{
    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    yarp::dev::PolyDriver m_analogDevice; /**< Analog device. */

    int m_noActuatedAxis; /**< Number of the actuated DoF */
    size_t m_noAnalogSensor; /**< Number of the joints ( associated with the analog sensors) */
    size_t m_noAllJoints; /**< Number of all the interested joints ( associated with the analog &
                             encoders sensors) */
    size_t m_noAllAxis;

    std::vector<std::string>
        m_actuatedAxisNames; /**< Vector containing the name of the controlled axes. */

    std::vector<std::string>
        m_allAxisNames; /**< Vector containing the name of the controlled axes. */

    std::vector<std::string>
        m_allJointNames; /**< Vector containing the name of the controlled axes. */

    std::vector<std::string>
        m_actuatedJointList; /**< Vector containing the names of the activated joints (activated
                                joints are related to the axis that we are using). */

    size_t m_noActuatedJoints;
    std::vector<axisSensorData>
        m_axisInfoList; /**< Vector containing the data structure for controlled axis and the
                           associated sensor list. */

    yarp::dev::IPreciselyTimed* m_timedInterface{nullptr};
    yarp::dev::IEncodersTimed* m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* m_positionDirectInterface{nullptr}; /**< Direct position control
                                                                       interface. */
    yarp::dev::IPositionControl* m_positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl* m_velocityInterface{nullptr}; /**< Velocity control interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits* m_limitsInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IAnalogSensor* m_AnalogSensorInterface{nullptr}; /**< Sensor interface */
    yarp::dev::ICurrentControl* m_currentInterface{nullptr}; /**< current control interface */
    yarp::dev::IPWMControl* m_pwmInterface{nullptr}; /**< PWM control interface*/
    yarp::dev::IPidControl* m_pidInterface{nullptr}; /**< pid control interface*/

    yarp::sig::Vector m_desiredJointValue; /**< Desired joint value [deg or deg/s]. */
    yarp::sig::Vector m_encoderPositionFeedbackInDegrees; /**< Joint position [deg]. */
    yarp::sig::Vector m_encoderPositionFeedbackInRadians; /**< Joint position [rad]. */
    yarp::sig::Vector m_encoderVelocityFeedbackInDegrees; /**< Axis velocities [deg/sec]. */
    yarp::sig::Vector m_encoderVelocityFeedbackInRadians; /**< Axis velocities [rad/sec]. */
    yarp::sig::Vector m_analogSensorFeedbackRaw; /**< sensor feedback [raw]*/
    yarp::sig::Vector m_analogSensorFeedbackInDegrees; /**< sensor feedback [deg]*/
    yarp::sig::Vector m_analogSensorFeedbackInRadians; /**< sensor feedback [rad]*/
    yarp::sig::Vector m_analogSensorFeedbackSelected; /**< sensor info to read*/
    yarp::sig::Vector m_SensorActuatedJointFeedbackInRadians; /**< all the interested sensor info to
                                                                 read (analog+encoders)*/
    yarp::sig::Vector m_currentFeedback; /**< motor current feedbacks*/
    yarp::sig::Vector m_desiredCurrent; /**< motor current reference*/
    yarp::sig::Vector m_desiredCurrentInterface; /**< motor current reference returned from the
                                                    Current interface*/
    yarp::sig::Vector m_pwmDesired; /**< motor PWM desires*/
    yarp::sig::Vector m_pwmFeedback; /**< motor PWM feedbacks*/
    yarp::sig::Vector
        m_pwmDesiredInterface; /**< motor PWM feedbacks returned from the PWM interface*/
    yarp::sig::Vector
        m_pidOutput; /**< low level pid controller output returned from the PID interface*/

    yarp::sig::Vector m_joints_min_boundary; /**< joint minimum possible value [deg]*/
    yarp::sig::Vector m_joints_max_boundary; /**< joint maximum possible value [deg]*/
    yarp::sig::Vector m_sensors_min_boundary; /**< senor minimum value [raw]*/
    yarp::sig::Vector m_sensors_max_boundary; /**< senor maximum value [raw]*/
    yarp::sig::Vector m_sensors_raw2Degree_scaling; /**< sacling from raw to Degree of joints*/

    yarp::os::Stamp m_timeStamp; /**< Time stamp. */

    bool m_isMandatory; /**< If false neglect the errors coming from the robot driver. */

    yarp::conf::vocab32_t m_controlMode; /**< Used control mode. */
    yarp::dev::PidControlTypeEnum m_pidControlMode; /**< Used pid control mode. */

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

    /**
     * Set the desired motor current
     * @param desiredCurrent desired motor currents
     * @return true / false in case of success / failure
     */
    bool setCurrentReferences(const yarp::sig::Vector& desiredCurrent);

    /**
     * Set the desired motor PWM
     * @param desiredPWM desired motor PWM
     * @return true / false in case of success / failure
     */
    bool setPwmReferences(const yarp::sig::Vector& desiredPWM);

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
     * Set the full interested joints feedback values
     * @return true / false in case of success / failure
     */
    bool setAllJointsFeedback();

    /**
     * Get the joint limits
     * @param limits matrix containing the joint limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(yarp::sig::Matrix& limits);

    /**
     * Get the joint limits
     * @param minLimits vector containing the joint minimum limits in radian
     * @param maxLimits vector containing the joint maximum limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(std::vector<double> minLimits, std::vector<double> maxLimits);

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
    const yarp::sig::Vector& axisFeedbacks() const;

    /**
     * Get the joint encoders value
     * @return the joint encoder value
     */
    void axisFeedbacks(std::vector<double>& axisFeedbacks) const;

    /**
     * Get the joint encoders speed
     * @return the joint encoder speed
     */
    const yarp::sig::Vector& jointEncodersSpeed() const;

    /**
     * Get the joint encoders speed
     * @return the joint encoder speed
     */
    void jointEncodersSpeed(std::vector<double>& axisVelocityFeedbacks) const;

    /**
     * Get the analog sensors value
     * @return the analog sensor values
     */
    const yarp::sig::Vector& analogSensors() const;

    /**
     * Get all the intersted joints sensors value (including analog+encoders)
     * @return all the intersted sensor values
     */
    const yarp::sig::Vector& allSensors() const;

    /**
     * Get all the intersted joints sensors value (including analog+encoders)
     * @return all the intersted sensor values
     */
    void allSensors(std::vector<double>& jointsFeedbacks) const;

    /**
     * Get all the intersted motor current values
     * @return all the motor current values
     */
    const yarp::sig::Vector& motorCurrents() const;

    /**
     * Get all the intersted motor current values
     * @return all the motor current values
     */
    void motorCurrents(std::vector<double>& motorCurrents) const;

    /**
     * Get all the actuated motor current References
     * @return all the motor current References
     */
    const yarp::sig::Vector& motorCurrentReference() const;

    /**
     * Get all the actuated motor current References
     * @return all the motor current References
     */
    void motorCurrentReference(std::vector<double>& motorCurrentReferences) const;

    /**
     * Get all the actuated motor PWM values
     * @return all the motor PWM values
     */
    const yarp::sig::Vector& motorPwm() const;

    /**
     * Get all the actuated motor PWM values
     * @return all the motor PWM values
     */
    void motorPwm(std::vector<double>& motorPwm) const;

    /**
     * Get all the actuated motor low level pid outputs
     * @return all the motor pid outputs
     */
    const yarp::sig::Vector& motorPidOutputs() const;

    /**
     * Get all the actuated motor low level pid outputs
     * @return all the motor pid outputs
     */
    void motorPidOutputs(std::vector<double>& motorPidOutputs) const;

    /**
     * Get all the actuated motor PWM References
     * @return all the motor PWM References
     */
    const yarp::sig::Vector& motorPwmReference() const;

    /**
     * Get all the actuated motor PWM References
     * @return all the motor PWM References
     */
    void motorPwmReference(std::vector<double>& motorPwmReference) const;

    /**
     * Get the number of actuated degree of freedom (motors)
     * @return the number of actuated DoF
     */
    const int getNumberOfActuatedAxis() const;

    const int getNumberOfAllAxis() const;

    /**
     * Get the number of joints
     * @return the number of joints
     */
    const int getNumberOfAllJoints() const;

    const int getNumberOfActuatedJoints() const;

    void getActuatedJointNames(std::vector<std::string>& names) const;

    void getAllJointNames(std::vector<std::string>& names) const;

    void getActuatedAxisNames(std::vector<std::string>& names) const;

    void getAllAxisNames(std::vector<std::string>& names) const;

    /**
     * Close the helper
     */
    bool close();
};

#endif
