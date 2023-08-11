// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ROBOT_MOTORS_ESTIMATION_HPP
#define ROBOT_MOTORS_ESTIMATION_HPP
// std
#include <memory>
#include <vector>

// yarp
#include <yarp/os/Property.h>

// teleoperation
#include <ControlHelper.hpp>
#include <MotorEstimation.hpp>

namespace HapticGlove
{
class Estimators;
} // namespace HapticGlove

/**
 * Estimators is a class for estimating the states of all the actuated axes/joints.
 */
class HapticGlove::Estimators
{

    std::string m_logPrefix;

    std::vector<Estimator> m_motorEstimatorVector; /// <summary> vector of motor/joint estimators

    size_t m_numOfMotors; /// <summary> number of motor/joint

    size_t m_n; /// <summary> number of states

    CtrlHelper::Eigen_Mat m_z; /// <summary> vector of motor/joint measurements

    bool m_isInitialized; /// <summary> estimators are initialized

    Eigen::VectorXd m_x_hat; /// <summary> vector of motor/joint expected values

    Eigen::VectorXd m_P; /// <summary>  vectorized version of the motor/joint estimation covariances

public:
    /**
     * constructor.
     * @param noMotors number of motors or joints
     */
    Estimators(const int noMotors);

    /**
     * destructor.
     */
    ~Estimators();

    /**
     * Configure the class
     * @param config configuration options
     * @param name name of the module
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name);

    /**
     * intialize the motor/joint estimators
     * @param  z0 initial measurements
     */
    bool initialize(const std::vector<double>& z0);

    /**
     * perform the estimatatiom step
     * @param  z measurements
     */
    bool estimateNextState(const std::vector<double>& z);

    /**
     * perform an estimation step assuming stochastic steady state system
     * @param z new measurement vector
     */
    bool estimateNextSteadyState(const std::vector<double>& z);

    /**
     * get the estimation results
     * @param estimatedValue expected angle value
     * @param estimatedVelocity expected velocity
     * @param estimatedAcceleration expected acceleration
     * @param P covariance of the estimated state
     */
    bool getInfo(Eigen::VectorXd& estimatedValue,
                 Eigen::VectorXd& estimatedVelocity,
                 Eigen::VectorXd& estimatedAcceleration,
                 Eigen::MatrixXd& P);

    /**
     * get the estimation results
     * @param estimatedValue expected angle value
     * @param estimatedVelocity expected velocity
     * @param estimatedAcceleration expected acceleration
     * @param P covariance of the estimated state
     */
    bool getInfo(std::vector<double>& estimatedValue,
                 std::vector<double>& estimatedVelocity,
                 std::vector<double>& estimatedAcceleration,
                 Eigen::MatrixXd& P);

    /**
     * get the estimation results of the expected axis/joint values
     * @param estimatedValue expected axis/joint value (angle)
     */
    bool getMotorValueInfo(std::vector<double>& estimatedValue);

    /**
     * check if the estimators are initialized
     */
    bool isInitialized() const;
};
#endif // ROBOT_MOTORS_ESTIMATION_HPP
