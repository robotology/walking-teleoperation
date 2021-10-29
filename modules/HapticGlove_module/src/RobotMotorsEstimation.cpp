/**
 * @file RobotMotorsEstimation.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <Utils.hpp>
#include <yarp/os/LogStream.h>

#include <RobotMotorsEstimation.hpp>

using namespace HapticGlove;

Estimators::Estimators(const int noMotors)
{
    m_numOfMotors = noMotors;
    m_isInitialized = false;
}

Estimators::~Estimators()
{
}

bool Estimators::configure(const yarp::os::Searchable& config, const std::string& name)
{

    size_t no_states_kf, no_measurement_kf;
    double dt;

    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", dt))
    {
        yError() << "[RobotMotorsEstimation::configure] Unable to find the samplingTime";
        return false;
    }

    no_states_kf = config.check("no_states_kf", yarp::os::Value(3)).asInt();
    no_measurement_kf = config.check("no_measurement_kf", yarp::os::Value(1)).asInt();

    yarp::sig::Vector Q_vector(no_states_kf, 0.0), R_vector(no_measurement_kf, 0.0);
    if (!YarpHelper::getYarpVectorFromSearchable(config, "r_matrix_kf", R_vector))
    {
        yError() << "[RobotController::configure] Initialization failed while reading "
                    "r_matrix_kf.";
        return false;
    }

    if (!YarpHelper::getYarpVectorFromSearchable(config, "q_matrix_kf", Q_vector))
    {
        yError() << "[RobotController::configure] Initialization failed while reading "
                    "q_matrix_kf.";
        return false;
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(no_states_kf, no_states_kf);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(no_measurement_kf, no_measurement_kf);
    for (int i = 0; i < no_states_kf; i++)
    {
        Q(i, i) = Q_vector(i);
    }
    for (int i = 0; i < no_measurement_kf; i++)
    {
        R(i, i) = R_vector(i);
    }

    for (int i = 0; i < m_numOfMotors; i++)
    {
        std::cout << "MotorEstimation initialization ...  \n";
        Estimator motorEstimator(dt, R, Q);
        m_motorEstimatorVector.push_back(motorEstimator);
    }
    std::cout << "All Motor Estimation are initialized. \n";

    m_z = Eigen::MatrixXd::Zero(no_measurement_kf, 1);

    m_x_hat = Eigen::MatrixXd::Zero(no_measurement_kf, 1);
    m_P;

    return true;
}

bool Estimators::initialize(const std::vector<double>& z0)
{
    for (size_t i = 0; i < m_numOfMotors; i++)
    {
        m_z(0, 0) = z0[i];
        m_motorEstimatorVector[i].initialize(m_z);
    }
    m_isInitialized = true;
    return true;
}

bool Estimators::estimateNextState(const std::vector<double>& z, std::vector<double>& x_hat)
{

    Eigen::MatrixXd x_hat_mat;
    for (size_t i = 0; i < m_numOfMotors; i++)
    {
        m_z(0, 0) = z[i];
        m_motorEstimatorVector[i].estimateNextState(m_z, m_x_hat);

        //        m_motorValueEstimation[i]=x_hat[0];
        //        m_motorVelocityEstimation[i]=x_hat[1];
        //        m_motorAccelerationEstimation[i]=x_hat[2];
    }
    return true;
}

bool Estimators::estimateNextState(const std::vector<double>& z)
{
    for (size_t i = 0; i < m_numOfMotors; i++)
    {
        m_z(0, 0) = z[i];
        m_motorEstimatorVector[i].estimateNextState(m_z);
    }

    return true;
}

bool Estimators::estimateNextSteadyState(const std::vector<double>& z)
{

    for (size_t i = 0; i < m_numOfMotors; i++)
    {
        m_z(0, 0) = z[i];
        m_motorEstimatorVector[i].estimateNextSteadyState(m_z);
    }
    return true;
}

bool Estimators::getInfo(Eigen::VectorXd& estimatedValues,
                         Eigen::VectorXd& estimatedVelocities,
                         Eigen::VectorXd& estimatedAccelerations,
                         Eigen::MatrixXd& P)
{
    if (estimatedValues.size() != m_numOfMotors)
    {
        estimatedValues.resize(m_numOfMotors, 0.0);
    }

    if (estimatedVelocities.size() != m_numOfMotors)
    {
        estimatedValues.resize(m_numOfMotors, 0.0);
    }

    if (estimatedAccelerations.size() != m_numOfMotors)
    {
        estimatedValues.resize(m_numOfMotors, 0.0);
    }

    if (P.rows() != m_numOfMotors && P.cols() == std::pow(m_n, 2))
    {
        P.resize(m_numOfMotors, std::pow(m_n, 2));
    }

    size_t i = 0;
    for (auto& estimator : m_motorEstimatorVector)
    {
        estimator.getInfo(m_x_hat, m_P);

        estimatedValues[i] = m_x_hat[0];
        estimatedVelocities[i] = m_x_hat[1];
        estimatedAccelerations[i] = m_x_hat[2];

        P.row(i) = m_P;
        i++;
    }

    return true;
}

bool Estimators::getInfo(std::vector<double>& estimatedValues,
                         std::vector<double>& estimatedVelocities,
                         std::vector<double>& estimatedAccelerations,
                         Eigen::MatrixXd& P)
{
    if (estimatedValues.size() != m_numOfMotors)
    {
        estimatedValues.resize(m_numOfMotors, 0.0);
    }

    if (estimatedVelocities.size() != m_numOfMotors)
    {
        estimatedValues.resize(m_numOfMotors, 0.0);
    }

    if (estimatedAccelerations.size() != m_numOfMotors)
    {
        estimatedValues.resize(m_numOfMotors, 0.0);
    }

    if (P.rows() != m_numOfMotors && P.cols() == std::pow(m_n, 2))
    {
        P.resize(m_numOfMotors, std::pow(m_n, 2));
    }

    size_t i = 0;
    for (auto& estimator : m_motorEstimatorVector)
    {
        estimator.getInfo(m_x_hat, m_P);

        estimatedValues[i] = m_x_hat[0];
        estimatedVelocities[i] = m_x_hat[1];
        estimatedAccelerations[i] = m_x_hat[2];

        P.row(i) = m_P;
        i++;
    }

    return true;
}

bool Estimators::getMotorValueInfo(std::vector<double>& estimatedValue)
{
    if (estimatedValue.size() != m_numOfMotors)
    {
        estimatedValue.resize(m_numOfMotors, 0.0);
    }

    size_t i = 0;
    for (auto& estimator : m_motorEstimatorVector)
    {
        estimator.getExpetedStateInfo(m_x_hat);
        estimatedValue[i] = m_x_hat[0];
        i++;
    }

    return true;
}

inline bool Estimators::isInitialized()
{
    return m_isInitialized;
}
