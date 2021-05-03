/**
 * @file RobotController_hapticGlove.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef ROBOT_CONTROLLER_HAPTIC_GLOVE_HPP
#define ROBOT_CONTROLLER_HAPTIC_GLOVE_HPP

// std
#include <iostream>
#include <memory>

// Eigen
#include <Eigen/Dense>

// YARP
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

#include <RobotControlHelper_hapticGlove.hpp>
#include <RobotMotorsEstimation.hpp>
#include <LinearRegression.hpp>

using namespace yarp::math;

/**
 * Class useful to manage the retargeting of fingers.
 */
class RobotController : public RobotControlHelper
{
private:
    yarp::sig::Vector m_fingersScaling; /**< It contains the finger velocity scaling. */

    std::unique_ptr<iCub::ctrl::Integrator> m_fingerIntegrator{nullptr}; /**< Velocity integrator */

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_A; /**< Coupling Matrix from the motors to the joints; Dimension <n,m> n: number of
                joints, m: number of motors; we have q= m_A x m + m_Bias where q is the joint values and m is
                the motor values*/

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_Bias; /**< Bias term of the coupling relationship from the motors to the joints; Dimension <n,1> n: number of
                joints; we have q= m_A x m  m_Bias where q is the joint values and m is
                the motor values*/


    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_Q; // weight matrix for desired states
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_R; // wieght for control output

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_controlCoeff; /**< control coeeficient matrix from the joints to the motors; Dimension
                <m,q> q: number of joints, m: number of motors*/

    bool m_doCalibration; /**< check if we need to do calibraition */
    bool m_motorJointsCoupled; /**< check if the motors and joints are coupled */

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_motorsData; /**< The logged data for calibration; the motors values; Dimension <o, m> o:
                         number of observations (logged data), m: number of motors */

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_jointsData; /**< The logged data for calibration; joints values; Dimension <o, n> o:
                         number of observations (logged data), n: number of joints */

    yarp::sig::Vector motorVelocityReference;

    std::unique_ptr<RobotMotorsEstimation> m_robotMotorFeedbackEstimator; /**< The KF estimated motor feedbacks*/
    std::unique_ptr<RobotMotorsEstimation> m_robotMotorReferenceEstimator; /**< The KF estimated motor References*/

    std::unique_ptr<RobotMotorsEstimation> m_robotJointFeedbackEstimator; /**< The KF estimated joint feedbacks*/
    std::unique_ptr<RobotMotorsEstimation> m_robotJointExpectedEstimator; /**< The KF estimated joint References*/

    std::unique_ptr<LinearRegression> m_linearRegressor;

    bool m_robotPrepared;


public:
    /**
     * Configure the object.
     * @param config reference to a resource finder object.
     * @param name name of the robot
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    /**
     * Set the fingers axis reference value
     * @param fingersReference the reference value for the finger axis to follow
     * @return true in case of success and false otherwise.
     */
    bool setFingersAxisReference(const yarp::sig::Vector& fingersReference);

    /**
     * Set the fingers joint reference value
     * @param fingersReference the reference value for the finger joints to follow
     * @return true in case of success and false otherwise.
     */
    bool setFingersJointReference(const yarp::sig::Vector& fingersReference);

    /**
     * Get the fingers' axis velocities or values
     * @param fingerValue get the fingers' axis velocity or value
     */
    void getFingerAxisFeedback(yarp::sig::Vector& fingerValues);

    /**
     * Get the fingers' axis velocities or values
     * @param fingerValue get the fingers' axis velocity or value
     */
    void getFingerAxisFeedback(std::vector<double>& fingerValues);

    /**
     * Get the fingers axis velocity feedback values
     * @param fingerAxisVelocityFeedback get the finger motor velocity feedback
     */
    void getFingerAxisVelocityFeedback(yarp::sig::Vector& fingerAxisVelocityFeedback);

    /**
     * Get the fingers axis velocity feedback values
     * @param fingerAxisVelocityFeedback get the finger motor velocity feedback
     */
    void getFingerAxisVelocityFeedback(std::vector<double>& fingerAxisVelocityFeedback);

    /**
     * Get the fingers joint velocities or values
     * @param fingerValue get the finger joint velocity or value
     */
    void getFingerJointsFeedback(yarp::sig::Vector& fingerValues);

    /**
     * Get the fingers joint velocities or values
     * @param fingerValue get the finger joint velocity or value
     */
    void getFingerJointsFeedback(std::vector<double>& fingerValues);

    /**
     * Get the motor current values
     * @param motorCurrentFeedback get the motor current feedback values
     */
    void getMotorCurrentFeedback(std::vector<double>& motorCurrentFeedback);

    /**
     * Get the motor current values
     * @param motorCurrentFeedback get the motor current feedback values
     */
    void getMotorCurrentFeedback(yarp::sig::Vector& motorCurrentFeedback);

    /**
     * Get the motor current values
     * @param motorCurrentFeedback get the motor current feedback values
     */
    void getMotorCurrentReference(std::vector<double>& motorCurrentReference);

    /**
     * Get the motor current values
     * @param motorCurrentFeedback get the motor current feedback values
     */
    void getMotorCurrentReference(yarp::sig::Vector& motorCurrentReference);

    /**
     * Get the fingers joint reference values
     * @param fingerJointsReference get the finger joint velocity or value
     */
    void getFingerAxisValueReference(yarp::sig::Vector& fingerAxisReference);

    /**
     * Get the fingers joint reference values
     * @param fingerJointsReference get the finger joint velocity or value
     */
    void getFingerAxisValueReference(std::vector<double>& fingerAxisReference);

    /**
     * Get the fingers joint velocities or values
     * @param fingerJointsReference get the finger joint velocity or value
     */
    void getFingerJointReference(yarp::sig::Vector& fingerJointsReference);

    /**
     * Get the fingers joint velocities or values
     * @param fingerJointsReference get the finger joint velocity or value
     */
    void getFingerJointReference(std::vector<double>& fingerJointsReference);

    /**
     * Get the fingers joint values computed from the axis feedback
     * @param fingerJointsExpectedValue get the finger joint value
     */
    void getFingerJointExpectedValue(yarp::sig::Vector& fingerJointsExpectedValue);

    /**
     * Get the fingers joint values computed from the axis feedback
     * @param fingerJointsExpectedValue get the finger joint value
     */
    void getFingerJointExpectedValue(std::vector<double>& fingerJointsExpectedValue);


    /**
     * Update the feedback values
     */
    bool updateFeedback(void);

    /**
     * Find the coupling relationship bwtween the motors and joitns of the robots
     * @return true if it could open the logger
     */
    bool LogDataToCalibrateRobotMotorsJointsCouplingRandom(const bool generateRandomVelocity);

    /**
     * Find the coupling relationship bwtween the motors and joitns of the robots using sin input
     * function
     * @param time the current time
     * @param axisNumber the axis to control and give input
     * @return true if it could open the logger
     */
    bool LogDataToCalibrateRobotMotorsJointsCouplingSin(double time, int axisNumber);

    /**
     * Solving the linear regression problem to compute the matrix m_A
     * @return true if it could open the logger
     */
    bool trainCouplingMatrix();
    /**
     * check if the robot is prepared
     * @return true if the robot is prepared
     */
    bool isRobotPrepared();

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
    bool getEstimatedMotorsState(std::vector<double>& feedbackAxisValuesEstimationKF, std::vector<double>&  feedbackAxisVelocitiesEstimationKF, std::vector<double>&  feedbackAxisAccelrationEstimationKF, Eigen::MatrixXd& feedbackAxisCovEstimationKF,
                                 std::vector<double>& referenceAxisValuesEstimationKF, std::vector<double>&  referenceAxisVelocitiesEstimationKF, std::vector<double>&  referenceAxisAccelrationEstimationKF, Eigen::MatrixXd& referenceAxisCovEstimationKF);


    /**
     * get joints estimated states
     * @return true if the robot joints estimator is returned correctly
     */
    bool getEstimatedJointState(std::vector<double>& feedbackJointValuesEstimationKF, std::vector<double>&  feedbackJointVelocitiesEstimationKF, std::vector<double>&  feedbackJointAccelrationEstimationKF, Eigen::MatrixXd& feedbackJointCovEstimationKF,
                                 std::vector<double>& expectedJointValuesEstimationKF, std::vector<double>&  expectedJointVelocitiesEstimationKF, std::vector<double>&  expectedJointAccelrationEstimationKF, Eigen::MatrixXd& expectedJointCovEstimationKF);

    /**
     * get joints estimated states
     * @return true if the robot joints estimator is returned correctly
     */
    bool getEstimatedJointState(std::vector<double>& feedbackJointValuesEstimationKF,
                                 std::vector<double>& expectedJointValuesEstimationKF);

};

#endif
