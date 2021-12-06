/**
 * @file RobotInterface.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

// std
#include <memory>
#include <unordered_map>
#include <vector>

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
class RobotInterface;
struct JointInfo;
} // namespace HapticGlove

struct HapticGlove::JointInfo
{
    bool useAnalog;
    size_t index; /**< The index of the data in analog data list or the axis list */
    double scale; /**< This parameter is used only when using axis encoders to compute the joint
                    angle. In this case, the encoder readouts are scaled to the robot joint values*/
};

/**
 * RobotControlInterface is an helper class for controlling the robot.
 */
class HapticGlove::RobotInterface
{
    std::string m_logPrefix; /**< the log prefix*/

    bool m_rightHand; /**< if the right hand is used, the variable is true*/

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
        m_actuatedJointList; /**< Vector containing the names of the actuated joints (activated
                                joints are related to the axis that we are using). */

    size_t m_noActuatedJoints;

    std::unordered_map<std::string, JointInfo> m_jointInfoMap;

    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    yarp::dev::PolyDriver m_analogDevice; /**< Analog device. */

    yarp::dev::IPreciselyTimed* m_timedInterface{nullptr};
    yarp::dev::IEncodersTimed* m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* m_positionDirectInterface{nullptr}; /**< Direct position control
                                                                       interface. */
    yarp::dev::IPositionControl* m_positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl* m_velocityInterface{nullptr}; /**< Velocity control interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits* m_limitsInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IAnalogSensor* m_analogSensorInterface{nullptr}; /**< Sensor interface */
    yarp::dev::ICurrentControl* m_currentInterface{nullptr}; /**< current control interface */
    yarp::dev::IPWMControl* m_pwmInterface{nullptr}; /**< PWM control interface*/
    yarp::dev::IPidControl* m_pidInterface{nullptr}; /**< pid control interface*/

    yarp::sig::Vector m_encoderPositionFeedbackInDegrees; /**< axis position [deg]. */
    yarp::sig::Vector m_encoderPositionFeedbackInRadians; /**< axis position [rad]. */
    yarp::sig::Vector m_encoderVelocityFeedbackInDegrees; /**< axis velocities [deg/sec]. */
    yarp::sig::Vector m_encoderVelocityFeedbackInRadians; /**< axis velocities [rad/sec]. */
    yarp::sig::Vector m_analogSensorFeedbackRaw; /**< analog sensor feedback [raw]*/
    yarp::sig::Vector m_analogSensorFeedbackInDegrees; /**< analog sensor feedback [deg]*/
    yarp::sig::Vector m_analogSensorFeedbackInRadians; /**< analog sensor feedback [rad]*/
    yarp::sig::Vector
        m_actuatedJointFeedbacksInRadian; /**< actuated joint feedback (analog+encoders)*/
    yarp::sig::Vector m_motorCurrentFeedbacks; /**< motor current feedbacks*/
    yarp::sig::Vector m_motorPwmFeedbacks; /**< motor PWM feedbacks*/
    yarp::sig::Vector
        m_pidOutput; /**< low level pid controller output returned from the PID interface*/

    yarp::sig::Vector m_referenceValues; /**< reference axis/motor values, depending on the
                                              control mode, may have different units */
    yarp::sig::Vector m_axisPositionReferences; /**< axis position reference [rad]. */
    yarp::sig::Vector m_axisPositionDirectReferences; /**< axis position [rad]. */
    yarp::sig::Vector m_axisVelocityReferences; /**< axis velocities [rad/sec]. */
    yarp::sig::Vector m_motorCurrentReferences; /**< motor current references*/
    yarp::sig::Vector m_motorPwmReferences; /**< motor PWM references*/

    std::vector<double> m_analogJointsMinBoundary; /**< joint minimum possible value [deg]*/
    std::vector<double> m_analogJointsMaxBoundary; /**< joint maximum possible value [deg]*/
    std::vector<double> m_analogSensorsRawMinBoundary; /**< senor minimum value [raw]*/
    std::vector<double> m_analogSensorsRawMaxBoundary; /**< senor maximum value [raw]*/
    std::vector<double> m_sensorsRaw2DegreeScaling; /**< sacling from raw to Degree of joints*/

    yarp::os::Stamp m_timeStamp; /**< Time stamp. */

    bool m_isMandatory; /**< If false neglect the errors coming from the robot driver. */

    yarp::conf::vocab32_t m_controlMode; /**< Used control mode. */
    yarp::dev::PidControlTypeEnum m_pidControlMode; /**< Used pid control mode. */

    double m_refenceVelocityForPositionControl;

    /**
     * Switch to control mode
     * @param controlMode is the specific control mode
     * @return true / false in case of success / failure
     */
    bool switchToControlMode(const int& controlMode);

    /**
     * Initialize the axis values to the min limits
     * @return true / false in case of success / failure
     */
    bool initializeAxesValues();

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

    /**
     * Set the desired joint reference (position or velocity)
     * @param desiredValue desired joint velocity or position (radiant or radiant/s)
     * @param controlMode the control mode
     * @return true / false in case of success / failure
     */
    bool setAxisReferences(std::vector<double>& desiredValues, const int& controlMode);

    /**
     * Get calibrated sensory feedback from the robot
     * @return true / false in case of success / failure
     */
    bool computeCalibratedAnalogSesnors();

    /**
     * Set the full interested joints feedback values
     * @return true / false in case of success / failure
     */
    bool computeActuatedJointFeedbacks();

    /**
     * Get the joint limits
     * @param limits matrix containing the joint limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(yarp::sig::Matrix& limits);

public:
    /**
     * Configure the helper
     * @param config confifuration options
     * @param name name of the robot
     * @param isMandatory if true the helper will return an error if there is a
     * problem in the configuration phase
     * @return true / false in case of success / failure
     */
    bool configure(const yarp::os::Searchable& config,
                   const std::string& name,
                   const bool& rightHand,
                   const bool& isMandatory);

    /**
     * Update the time stamp
     */
    void updateTimeStamp();

    /**
     * Set the desired joint reference (position or velocity)
     * @param desiredValue desired joint velocity or position (radiant or radiant/s)
     * @return true / false in case of success / failure
     */
    bool setAxisReferences(std::vector<double>& desiredValues);

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
     * @param minLimits vector containing the joint minimum limits in radian
     * @param maxLimits vector containing the joint maximum limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(std::vector<double>& minLimits, std::vector<double>& maxLimits);

    /**
     * Get the joint limits
     * @param minLimits yarp vector containing the joint minimum limits in radian
     * @param maxLimits yarp vector containing the joint maximum limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(yarp::sig::Vector& minLimits, yarp::sig::Vector& maxLimits);

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
     * Get the axis encider values
     * @return the axis encoder value
     */
    const yarp::sig::Vector& axisFeedbacks() const;

    /**
     * Get the axis encoders value
     * @param axisFeedbacks the axis encoder values
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
    void jointEncodersSpeed(std::vector<double>& axisVelocityFeedbacks);

    /**
     * Get the analog sensors value
     * @return the analog sensor values
     */
    const yarp::sig::Vector& analogSensors() const;

    /**
     * Get all the intersted joints sensors value (including analog+encoders)
     * @return all the intersted sensor values
     */
    const yarp::sig::Vector& actuatedJointFeedbacks() const;

    /**
     * Get all the intersted joints sensors value (including analog+encoders)
     * @param jointsFeedbacks all the intersted sensor values
     */
    void actuatedJointFeedbacks(std::vector<double>& jointsFeedbacks);

    /**
     * Get all the actuated motor current values
     * @return all the motor current values
     */
    const yarp::sig::Vector& motorCurrents() const;

    /**
     * Get all the actuated motor current values
     * @param  motorCurrents the motor current values
     */
    void motorCurrents(std::vector<double>& motorCurrents);

    /**
     * Get all the actuated motor current References
     * @return all the motor current References
     */
    const yarp::sig::Vector& motorCurrentReference() const;

    /**
     * Get all the actuated motor current References
     * @return all the motor current References
     */
    void motorCurrentReference(std::vector<double>& motorCurrentReferences);

    /**
     * Get all the actuated motor PWM values
     * @return all the motor PWM values
     */
    const yarp::sig::Vector& motorPwm() const;

    /**
     * Get all the actuated motor PWM values
     * @return all the motor PWM values
     */
    void motorPwm(std::vector<double>& motorPwm);

    /**
     * Get all the actuated motor low level pid outputs
     * @return all the motor pid outputs
     */
    const yarp::sig::Vector& motorPidOutputs() const;

    /**
     * Get all the actuated motor low level pid outputs
     * @param motorPidOutputs the motor pid outputs
     */
    void motorPidOutputs(std::vector<double>& motorPidOutputs);

    /**
     * Get all the actuated motor PWM References
     * @return all the motor PWM References
     */
    const yarp::sig::Vector& motorPwmReference() const;

    /**
     * Get all the actuated motor PWM References
     * @param motorPwmReference the motor PWM References
     */
    void motorPwmReference(std::vector<double>& motorPwmReference);

    /**
     * Get the number of actuated axis/motors
     * @return the number of actuated axis/motors
     */
    const int getNumberOfActuatedAxis() const;

    /**
     * Get the number of all axis/motors related to the used parts
     * @return the number of all axis/motors
     */
    const int getNumberOfAllAxis() const;

    /**
     * Get the number of all the joints related to the robot used parts
     * @return the number of all the joints
     */
    const int getNumberOfAllJoints() const;

    /**
     * Get the number of actuated joints of the used parts
     * @return the number of actuated joints
     */
    const int getNumberOfActuatedJoints() const;

    /**
     * Get the name of the actuated joints
     * @param names the names of the actuated joints
     */
    void getActuatedJointNames(std::vector<std::string>& names) const;

    /**
     * Get the name of the all the joints related to the used parts
     * @param names the names of the all joints
     */
    void getAllJointNames(std::vector<std::string>& names) const;

    /**
     * Get the name of the actuated axes
     * @param names the names of the actuated axes
     */
    void getActuatedAxisNames(std::vector<std::string>& names) const;

    /**
     * Get the name of the all the axes related to the used parts
     * @param names the names of the all axis
     */
    void getAllAxisNames(std::vector<std::string>& names) const;

    /**
     * Close the helper
     */
    bool close();
};

#endif
