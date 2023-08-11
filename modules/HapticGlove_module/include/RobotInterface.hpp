// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

// std
#include <array>
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
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

namespace HapticGlove
{
class RobotInterface;
struct JointInfo;
} // namespace HapticGlove

/**
 * JointInfo is a class for retrieving the information related to a joint.
 */
struct HapticGlove::JointInfo
{
    bool useAnalog; /**< if the joint feedback is obtained from the analog sensor or from the axis
                       encoder (in the case a robot hand joint does not have analog/hall sensor, we
                       use the related encoder)*/
    size_t index; /**< The index of the data in analog data list or in the axis list depending on
                     the useAnalog flag*/
    double scale; /**< This parameter is used only when using axis encoders to compute the joint
                    angle. In this case, the encoder readouts are scaled to the robot joint values.
                    This is mainly useful when single axis encoder value is used to specify several
                    joint angle value due to the lack of sensors (e.g., l_hand_finger or
                    r_hand_finger associated joints).*/
};

/**
 * RobotInterface is a class for interfacing with the robot, controling the robot, and getting
 * feedback from the robot.
 */
class HapticGlove::RobotInterface
{
    std::string m_logPrefix; /**< the log prefix*/

    bool m_rightHand; /**< if the right hand is used, the variable is true*/

    int m_noActuatedAxis; /**< Number of the actuated axis of the robot hand */
    size_t
        m_noAnalogSensor; /**< Number of the analog joints ( associated with the analog sensors) */
    size_t m_noAllJoints; /**< Number of all the robot hand joints ( associated with the analog &
                             encoders sensors) */
    size_t m_noAllAxis; /**< the number of all robot hand axis */

    size_t m_noFingers; /**< the number of robot fingers */

    std::vector<std::string>
        m_robotFingerNames; /**< Vector containing the name of the all the robot hand fingers. */

    std::vector<std::string> m_actuatedAxisNames; /**< Vector containing the name of the controlled
                                                     axes of the robot hand. */

    std::vector<std::string>
        m_allAxisNames; /**< Vector containing the name of the all hand axes. */

    std::vector<std::string>
        m_allJointNames; /**< Vector containing the name of the all the robot hand joint names. */

    std::vector<std::string>
        m_actuatedJointList; /**< Vector containing the names of the actuated joints (activated
                                joints are related to the axis that are actauted). */

    size_t m_noActuatedJoints; /**< number of actuated joints. */

    std::unordered_map<std::string, JointInfo>
        m_jointInfoMap; /**< the unordered map from the joint name to the joint info*/

    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    yarp::dev::PolyDriver m_analogDevice; /**< Analog device. */

    yarp::dev::IPreciselyTimed* m_timedInterface{nullptr}; /**< Time interface. */
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

    yarp::sig::Vector m_analogJointsMinBoundaryDegree; /**< joint minimum possible value [deg]*/
    yarp::sig::Vector m_analogJointsMaxBoundaryDegree; /**< joint maximum possible value [deg]*/
    yarp::sig::Vector m_analogSensorsRawMinBoundary; /**< senor minimum value [raw]*/
    yarp::sig::Vector m_analogSensorsRawMaxBoundary; /**< senor maximum value [raw]*/
    yarp::sig::Vector
        m_sensorsRaw2DegreeScaling; /**< sacling from raw to Degree of joints with analog readouts*/

    yarp::sig::Matrix m_actuatedAxisLimits; /**< the min and max limits of the actuated axis */

    std::unordered_map<std::string, std::array<double, 2>>
        m_axisCustomMotionRange; /**< the unordered map from the axis name to the custom range of
                                    motion of the axis limit; this will overwrite the
                                    m_actuatedAxisLimits in [rad] */

    std::unordered_map<std::string, double>
        m_axisCustomHomeValues; /**< the unordered map from the axis name to the custom axis homing
                                  value in [rad] */

    yarp::sig::Vector
        m_actuatedAxisHomeValues; /**< the vector of actuated axis home pose in [rad] */

    yarp::os::Stamp m_timeStamp; /**< Time stamp. */

    bool m_isMandatory; /**< If false neglect the errors coming from the robot driver. */

    yarp::conf::vocab32_t m_controlMode; /**< Used control mode. */

    yarp::dev::PidControlTypeEnum m_pidControlMode; /**< Used pid control mode. */

    double m_steadyStateThreshold;

    size_t m_steadyStateCounter;

    size_t m_steadyStateCounterThreshold;

    /**
     * Switch to control mode
     * @param controlMode is the specific control mode
     * @return true / false in case of success / failure
     */
    bool switchToControlMode(const int& controlMode);

    /**
     * Initialize the axis values to the min limits
     * @param initializationTime the time necessary to initialize the robot (default value is 5
     * seconds) [sec]
     * @return true / false in case of success / failure
     */
    bool initializeAxisValues(const double& initializationTime = 5.0);

    /**
     * Set the desired axis position direct (position direct mode)
     * @param desiredPosition desired axis position in radiant
     * @return true / false in case of success / failure
     */
    bool setDirectPositionReferences(const yarp::sig::Vector& desiredPositions);

    /**
     * Set the desired axis position (min jerk trajectory mode)
     * @param desiredPosition desired axis position in radiant
     * @return true / false in case of success / failure
     */
    bool setPositionReferences(const yarp::sig::Vector& desiredPositions);

    /**
     * Set the desired axis velocity
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
     * Set the desired axis reference
     * @param desiredValue desired axis value
     * @param controlMode the control mode
     * @return true / false in case of success / failure
     */
    bool setAxisReferences(std::vector<double>& desiredValues, const int& controlMode);

    /**
     * Get calibrated analog sensory feedback from the robot
     * @return true / false in case of success / failure
     */
    bool computeCalibratedAnalogSesnors();

    /**
     * Compute the full actuated joint feedback values
     * @return true / false in case of success / failure
     */
    bool computeActuatedJointFeedbacks();

    /**
     * Get the axis limits
     * @param limits matrix containing the axis limits in radian
     * @return true / false in case of success / failure
     */
    bool getActuatedAxisLimits(yarp::sig::Matrix& limits);

    /**
     * Check if the axis values reached the desireed axis values and is steady state
     * @param reference reference values to the robot
     * @param feedback feedback values received from the robot
     * @return true / false in case of success / failure
     */
    bool isSteadyStateReached(yarp::sig::Vector& reference, yarp::sig::Vector& feedback);

    /**
     * Check if the axis values reached the desireed axis values and is steady state
     * @param reference reference values to the robot
     * @param feedback feedback values received from the robot
     * @return true / false in case of success / failure
     */
    bool isSteadyStateReached(std::vector<double>& reference, std::vector<double>& feedback);

    /**
     * Compute the actuated axis home pose values
     * @return true / false in case of success / failure
     */
    bool computeActuatedAxisHomeValues();

public:
    /**
     * Configure the robot interface
     * @param config confifuration options
     * @param name name of the robot
     * @param rightHand if the right hand of the robot is used
     * @param isMandatory if true the robot interface will return an error if there is a
     * problem in the configuration phase
     * @return true / false in case of success / failure
     */
    bool configure(const yarp::os::Searchable& config,
                   const std::string& name,
                   const bool& rightHand,
                   const bool& isMandatory);

    /**
     * Open robot devices
     * @return true / false in case of success / failure
     */
    bool openRobotDevices(const yarp::os::Searchable& config,
                          const std::string& name,
                          const std::string& robot);

    /**
     * Open analog devices
     * @return true / false in case of success / failure
     */
    bool openAnalogDevices(const yarp::os::Searchable& config,
                           const std::string& name,
                           const std::string& robot);

    /**
     * Update the time stamp
     */
    void updateTimeStamp();

    /**
     * Set the desired axis reference (position/positionDirect/velocity/pwm/current)
     * depending on the control mode is specified and read from the configuration files
     * @param desiredValue desired joint velocity or position (unit depends on the control mode)
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
     * Get the axis limits
     * @param minLimits vector containing the axis minimum limits in radian
     * @param maxLimits vector containing the axis maximum limits in radian
     * @return true / false in case of success / failure
     */
    bool getActuatedAxisLimits(std::vector<double>& minLimits, std::vector<double>& maxLimits);

    /**
     * Get the axis limits
     * @param minLimits yarp vector containing the axis minimum limits in radian
     * @param maxLimits yarp vector containing the axis maximum limits in radian
     * @return true / false in case of success / failure
     */
    bool getActuatedAxisLimits(yarp::sig::Vector& minLimits, yarp::sig::Vector& maxLimits);

    /**
     * Get the axis home Values
     * @param axisHomeValues vector containing the axis homing values in radian
     * @return true / false in case of success / failure
     */
    bool getActuatedAxisHomeValues(std::vector<double>& axisHomeValues);

    /**
     * Get the axis home Values
     * @param axisHomeValues vector containing the axis homing values in radian
     * @return true / false in case of success / failure
     */
    bool getActuatedAxisHomeValues(yarp::sig::Vector& axisHomeValues);

    /**
     * Get the axis velocity limits
     * @param limits matrix containing the axis velocity limits in radian/sec
     * @return true / false in case of success / failure
     */
    bool getVelocityLimits(yarp::sig::Matrix& limits);

    /**
     * Get the actuated joint limits
     * @param minLimits vector containing the joint minimum limits in radian
     * @param maxLimits vector containing the joint maximum limits in radian
     * @return true / false in case of success / failure
     */
    bool getActuatedJointLimits(std::vector<double>& minLimits, std::vector<double>& maxLimits);

    /**
     * Get the time stamp (const version)
     * @return time stamp
     */
    const yarp::os::Stamp& timeStamp() const;

    /**
     * Get the actuated axis encoder values
     * @return the actuated axis encoder value
     */
    const yarp::sig::Vector& axisFeedbacks() const;

    /**
     * Get the actuated axis encoders value
     * @param axisFeedbacks the actuated axis encoder values
     */
    void axisFeedbacks(std::vector<double>& axisFeedbacks);

    /**
     * Get the actuated axis position references
     * @return  actauted axis position references
     */
    const yarp::sig::Vector& axisPositionReferences() const;

    /**
     * Get the actuated axis position references
     * @param axisPositionReferences actauted axis position references
     */
    void axisPositionReferences(std::vector<double>& axisPositionReferences);

    /**
     * Get the actuated axis position direct references
     * @return actauted axis position direct references
     */
    const yarp::sig::Vector& axisPositionDirectReferences() const;

    /**
     * Get the actuated position direct References
     * @param axisPositionDirectReferences actauted axis position direct references
     */
    void axisPositionDirectReferences(std::vector<double>& axisPositionDirectReferences);

    /**
     * Get the axis encoders velocity vector
     * @return the actuated axis encoder speed
     */
    const yarp::sig::Vector& axisVelocityFeedbacks() const;

    /**
     * Get the axis encoders velocity vector
     * @param axisVelocityFeedbacks the actuated axis encoder velocity vector
     */
    void axisVelocityFeedbacks(std::vector<double>& axisVelocityFeedbacks);

    /**
     * Get the actuated axis velocity references
     * @return actauted axis velocity references
     */
    const yarp::sig::Vector& axisVelocityReferences() const;

    /**
     * Get the actuated axis velocity References
     * @param axisVelocityReferences actauted axis velocity references
     */
    void axisVelocityReferences(std::vector<double>& axisVelocityReferences);

    /**
     * Get the analog sensors value
     * @return the analog sensor values
     */
    const yarp::sig::Vector& analogSensorFeedbacks() const;

    /**
     * Get the actuated joints sensors value (including analog+encoders)
     * @return actuated joint feedback values
     */
    const yarp::sig::Vector& actuatedJointFeedbacks() const;

    /**
     * Get the actuated joints sensors value (including analog+encoders)
     * @param jointsFeedbacks actuated joint feedback values
     */
    void actuatedJointFeedbacks(std::vector<double>& jointsFeedbacks);

    /**
     * Get the actuated motor current values
     * @return the actuated motor current values
     */
    const yarp::sig::Vector& motorCurrents() const;

    /**
     * Get the actuated motor current values
     * @param  motorCurrents the actuated motor current values
     */
    void motorCurrents(std::vector<double>& motorCurrents);

    /**
     * Get the actuated motor current References
     * @return actuated the motor current References
     */
    const yarp::sig::Vector& motorCurrentReference() const;

    /**
     * Get the actuated motor current References
     * @param motorCurrentReferences actuated motor current References
     */
    void motorCurrentReference(std::vector<double>& motorCurrentReferences);

    /**
     * Get the actuated motor PWM values
     * @return actuated the motor PWM values
     */
    const yarp::sig::Vector& motorPwm() const;

    /**
     * Get the actuated motor PWM values
     * @param motorPwm actuated motor PWM values
     */
    void motorPwm(std::vector<double>& motorPwm);

    /**
     * Get the actuated motor PWM References
     * @return actuated the motor PWM References
     */
    const yarp::sig::Vector& motorPwmReference() const;

    /**
     * Get the actuated motor PWM References
     * @param motorPwmReference the motor PWM References
     */
    void motorPwmReference(std::vector<double>& motorPwmReference);

    /**
     * Get the actuated motor low-level pid outputs
     * @return actuated motor pid outputs
     */
    const yarp::sig::Vector& motorPidOutputs() const;

    /**
     * Get the actuated motor low-level pid outputs
     * @param motorPidOutputs the motor pid outputs
     */
    void motorPidOutputs(std::vector<double>& motorPidOutputs);

    /**
     * Get the number of actuated axis/motors
     * @return the number of actuated axis/motors
     */
    const int getNumberOfActuatedAxis() const;

    /**
     * Get the number of all axis/motors related to the used parts (robot hand)
     * @return the number of all axis/motors
     */
    const int getNumberOfAllAxis() const;

    /**
     * Get the number of all the joints related to the robot used parts (robot hand)
     * @return the number of all the joints
     */
    const int getNumberOfAllJoints() const;

    /**
     * Get the number of actuated joints of the used parts (robot hand)
     * @return the number of actuated joints
     */
    const int getNumberOfActuatedJoints() const;

    /**
     * Get the number of iCub robot hand fingers
     * @return the number of icub robot hand fingers
     */
    const int getNumberOfRobotFingers() const;

    /**
     * Get the name of the robot fingers
     * @param names the names of the robot hand fingers
     */
    void getFingerNames(std::vector<std::string>& names) const;

    /**
     * Get the name of the actuated joints
     * @param names the names of the actuated joints
     */
    void getActuatedJointNames(std::vector<std::string>& names) const;

    /**
     * Get the name of the all the joints related to the used parts (robot hand)
     * @param names the names of the all joints
     */
    void getAllJointNames(std::vector<std::string>& names) const;

    /**
     * Get the name of the actuated axes
     * @param names the names of the actuated axes
     */
    void getActuatedAxisNames(std::vector<std::string>& names) const;

    /**
     * Get the name of the all the axes related to the used parts (robot hand)
     * @param names the names of the all axis
     */
    void getAllAxisNames(std::vector<std::string>& names) const;

    /**
     * Close the robot interface
     * @return true / false in case of success / failure
     */
    bool close();
};

#endif // ROBOT_INTERFACE_HPP
