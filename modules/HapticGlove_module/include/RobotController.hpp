// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

// std
#include <iostream>
#include <memory>

// teleoperation
#include <ControlHelper.hpp>
#include <LinearRegression.hpp>
#include <RobotInterface.hpp>
#include <RobotMotorsEstimation.hpp>

// yarp
#include <yarp/os/Searchable.h>

namespace HapticGlove
{
class RobotController;
} // namespace HapticGlove

/**
 * RobotController Class useful to manage the control of the retargeting robot fingers.
 */
class HapticGlove::RobotController
{
private:
    std::string m_logPrefix;

    bool m_robotPrepared;

    bool m_rightHand;

    bool m_estimatorsInitialized;

    bool m_doCalibration; /**< check if we need to do calibraition */

    bool m_axesJointsCoupled; /**< check if the axis and joints are coupled */

    size_t m_numAllAxis;
    size_t m_numActuatedAxis;
    size_t m_numAllJoints;
    size_t m_numActuatedJoints;

    std::unique_ptr<RobotInterface> m_robotInterface; /**< robot control interface */

    CtrlHelper::Eigen_Mat m_A; /**< Coupling Matrix from the motors to the joints; Dimension <n,m>
                            n: number of joints, m: number of motors; we have q= m_A x m + m_Bias
                            where q is the joint values and m is the motor values*/

    CtrlHelper::Eigen_Mat m_Bias; /**< Bias term of the coupling relationship from the motors to the
                            joints; Dimension <n,1> n: number of joints; we have q= m_A x m+m_Bias
                            where q is the joint values and m is the motor values*/

    CtrlHelper::Eigen_Mat m_Q; // weight matrix for desired states

    CtrlHelper::Eigen_Mat m_R; // wieght for control output

    CtrlHelper::Eigen_Mat m_controlCoeff; /**< control coeeficient matrix from the joints to the
                            motors; Dimension <m,q> q: number of joints, m: number of motors*/

    CtrlHelper::Eigen_Mat
        m_axesData; /**< The logged data for calibration; the motors values; Dimension <o, m> o:
                         number of observations (logged data), m: number of motors */

    CtrlHelper::Eigen_Mat
        m_jointsData; /**< The logged data for calibration; joints values; Dimension <o, n> o:
                         number of observations (logged data), n: number of joints */

    std::unique_ptr<Estimators> m_axisFeedbackEstimators; /**< The KF estimated motor feedbacks*/
    std::unique_ptr<Estimators> m_axisReferenceEstimators; /**< The KF estimated motor References*/

    std::unique_ptr<Estimators> m_jointFeedbackEstimators; /**< The KF estimated joint feedbacks*/
    std::unique_ptr<Estimators> m_jointExpectedEstimators; /**< The KF estimated joint References*/

    std::unique_ptr<LinearRegression> m_linearRegressor;

    double m_kGain; /**< The gain of the exponential filter to set the robot reference position
                       values */

    std::unique_ptr<CtrlHelper::Data> m_data;

    /**
     * get the custom set of vectors
     * @param allListName the full vector names
     * @param customListNames the custm members of the vector names
     * @param allListVector the full vector values
     * @param customListVector the custm members of the vector values
     * @return true/false in case of success/failure
     */
    bool getCustomSetIndices(const std::vector<std::string>& allListName,
                             const std::vector<std::string>& customListNames,
                             const std::vector<double>& allListVector,
                             std::vector<double>& customListVector);

public:
    /**
     * Configure the object.
     * @param config reference to a resource finder object.
     * @param name name of the robot
     * @return true in case of success and false otherwise.
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Set the fingers axis reference value
     * @param fingersReference the reference value for the finger axis to follow
     * @return true in case of success and false otherwise.
     */
    bool setAxisReferences(const std::vector<double>& axisReferences);

    /**
     * Set the fingers axis reference value
     * @param fingersReference the reference value for the finger axis to follow
     * @return true in case of success and false otherwise.
     */
    bool setAxisReferences(const Eigen::VectorXd& axisReferences);

    /**
     * Set the fingers joint reference value
     * @param fingersReference the reference value for the finger joints to follow
     * @return true in case of success and false otherwise.
     */
    bool setJointReferences(const std::vector<double>& fingersReferences);

    /**
     * Perform computations to find the control signals
     * @return true in case of success and false otherwise.
     */
    bool computeControlSignals();

    /**
     * Get the fingers' axis velocities or values
     * @param fingerValue get the fingers' axis velocity or value
     */
    void getAxisValueFeedbacks(std::vector<double>& fingerValues);

    /**
     * Get the fingers axis velocity feedback values
     * @param fingerAxisVelocityFeedback get the finger motor velocity feedback
     */
    void getAxisVelocityFeedbacks(std::vector<double>& fingerAxisVelocityFeedback);

    /**
     * Get the fingers joint velocities or values
     * @param fingerValue get the finger joint velocity or value
     */
    void getJointValueFeedbacks(std::vector<double>& fingerValues);

    /**
     * Get the motor current Feedback values
     * @param motorCurrentFeedback get the motor current feedback values
     */
    void getMotorCurrentFeedback(std::vector<double>& motorCurrentFeedback);

    /**
     * Get the motor current Refernce values
     * @param motorCurrentReference get the motor current reference values
     */
    void getMotorCurrentReference(std::vector<double>& motorCurrentReference);

    /**
     * Get the motor PWM values
     * @param motorPWMtFeedback get the motor PWM feedback values
     */
    void getMotorPwmFeedback(std::vector<double>& motorPWMFeedback);

    /**
     * Get the motor pid outputs
     * @param motorPidOutputs get the motor pid output values
     */
    void getMotorPidOutputs(std::vector<double>& motorPidOutputs);

    /**
     * Get the motor PWM Reference values
     * @param motorPWMReference get the motor PWM reference values
     */
    void getMotorPwmReference(std::vector<double>& motorPWMReference);

    /**
     * Get the fingers joint reference values
     * @param fingerJointsReference get the finger joint velocity or value
     */
    void getAxisValueReferences(std::vector<double>& axisReferences);

    /**
     * Get the fingers joint velocities or values
     * @param fingerJointsReference get the finger joint velocity or value
     */
    void getJointReferences(std::vector<double>& fingerJointsReference);

    /**
     * Get the fingers joint values computed from the axis feedback
     * @param fingerJointsExpectedValue get the finger joint value
     */
    void getJointExpectedValues(std::vector<double>& jointsValuesExpected);

    /**
     * Update the feedback values
     */
    bool updateFeedback(void);

    /**
     * Find the coupling relationship bwtween the motors and joitns of the robots using sin input
     * function
     * @param time the current time
     * @param axisNumber the axis to control and give input
     * @return true if it could open the logger
     */
    bool LogDataToCalibrateRobotAxesJointsCoupling(double time, int axisNumber);

    /**
     * Solving the linear regression problem to compute the matrix m_A
     * @return true if it could open the logger
     */
    bool trainCouplingMatrix();
    /**
     * check if the robot is prepared
     * @return true if the robot is prepared
     */
    bool isRobotPrepared() const;

    /**
     * check if estimators are initialized
     * @return true if estimators are initialized
     */
    bool areEstimatorsInitialized() const;

    /**
     * initialize the estimator of the motors
     * @return true if the robot motor estimator is initialized
     */
    bool initializeEstimators();

    /**
     * estimate the next states of the motor
     * @return true if the robot motor estimator is done correctly
     */
    bool estimateNextStates();

    /**
     * get motor estimated states
     * @return true if the robot motor estimator is returned correctly
     */
    bool getEstimatedMotorsState(std::vector<double>& feedbackAxisValuesEstimationKF,
                                 std::vector<double>& feedbackAxisVelocitiesEstimationKF,
                                 std::vector<double>& feedbackAxisAccelrationEstimationKF,
                                 Eigen::MatrixXd& feedbackAxisCovEstimationKF,
                                 std::vector<double>& referenceAxisValuesEstimationKF,
                                 std::vector<double>& referenceAxisVelocitiesEstimationKF,
                                 std::vector<double>& referenceAxisAccelrationEstimationKF,
                                 Eigen::MatrixXd& referenceAxisCovEstimationKF);

    /**
     * get joints estimated states
     * @return true if the robot joints estimator is returned correctly
     */
    bool getEstimatedJointValuesKf(std::vector<double>& feedbackJointValuesEstimationKF,
                                   std::vector<double>& feedbackJointVelocitiesEstimationKF,
                                   std::vector<double>& feedbackJointAccelrationEstimationKF,
                                   Eigen::MatrixXd& feedbackJointCovEstimationKF,
                                   std::vector<double>& expectedJointValuesEstimationKF,
                                   std::vector<double>& expectedJointVelocitiesEstimationKF,
                                   std::vector<double>& expectedJointAccelrationEstimationKF,
                                   Eigen::MatrixXd& expectedJointCovEstimationKF);

    /**
     * get joints estimated states
     * @return true if the robot joints estimator is returned correctly
     */
    bool getEstimatedJointValuesKf(std::vector<double>& feedbackJointValuesEstimationKF,
                                   std::vector<double>& expectedJointValuesEstimationKF);

    /**
     * Move the robot part
     * @return true in case of success and false otherwise.
     */
    bool move();

    /**
     * Expose the contolHelper interface (const)
     * @return control helper interface
     */
    const std::unique_ptr<RobotInterface>& controlHelper() const;

    /**
     * Expose the contolHelper interface
     * @return control helper interface
     */
    std::unique_ptr<RobotInterface>& controlHelper();
};

#endif // ROBOT_CONTROLLER_HPP
