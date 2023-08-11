// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <Utils.hpp>
#include <yarp/os/LogStream.h>

#include <RobotMotorsEstimation.hpp>

using namespace HapticGlove;

Estimators::Estimators(const int noMotors)
{
    m_numOfMotors = noMotors;
    m_isInitialized = false;
}

Estimators::~Estimators() = default;

bool Estimators::configure(const yarp::os::Searchable& config, const std::string& name)
{

    m_logPrefix = name + "::";
    m_logPrefix += "Estimators:: ";

    size_t no_states_kf, no_measurement_kf;
    double dt;

    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", dt))
    {
        yError() << m_logPrefix << "unable to find the samplingTime";
        return false;
    }

    no_states_kf = config.check("no_states_kf", yarp::os::Value(3)).asInt32();

    no_measurement_kf = config.check("no_measurement_kf", yarp::os::Value(1)).asInt32();

    yarp::sig::Vector Q_vector(no_states_kf, 0.0), R_vector(no_measurement_kf, 0.0);
    if (!YarpHelper::getYarpVectorFromSearchable(config, "r_matrix_kf", R_vector))
    {
        yError() << m_logPrefix << "Initialization failed while reading r_matrix_kf.";
        return false;
    }

    if (!YarpHelper::getYarpVectorFromSearchable(config, "q_matrix_kf", Q_vector))
    {
        yError() << m_logPrefix << "Initialization failed while reading q_matrix_kf.";
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
        Estimator motorEstimator(dt, R, Q);
        m_motorEstimatorVector.push_back(motorEstimator);
    }

    m_z = Eigen::MatrixXd::Zero(no_measurement_kf, 1);

    m_x_hat = Eigen::MatrixXd::Zero(no_measurement_kf, 1);
    m_P.resize(no_states_kf * no_states_kf);

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

    if (P.rows() != m_numOfMotors && P.cols() == m_n * m_n)
    {
        P.resize(m_numOfMotors, m_n * m_n);
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

    if (P.rows() != m_numOfMotors && P.cols() == m_n * m_n)
    {
        P.resize(m_numOfMotors, m_n * m_n);
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

bool Estimators::isInitialized() const
{
    return m_isInitialized;
}
