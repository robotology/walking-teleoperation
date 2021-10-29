/**
 * @file MotorEstimation.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <MotorEstimation.hpp>
#include <iostream>
using namespace HapticGlove;

Estimator::Estimator(const double& dt, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q)
{
    // hard coded model
    // reference issue: https://github.com/ami-iit/element_retargeting-from-human/issues/145
    m_n = 3; // joint/motor position, velocity, acceleration
    m_m = 3;
    m_p = 1;

    m_dt = dt;

    m_F = Eigen::MatrixXd::Zero(m_n, m_n);
    m_F(0, 1) = 1.0;
    m_F(1, 2) = 1.0;
    m_G = Eigen::MatrixXd::Identity(m_n, m_m);
    m_H = Eigen::MatrixXd::Zero(m_p, m_n);
    m_H(0, 0) = 1.0;

    m_R = R;
    m_Q = Q;

    m_kf = std::make_unique<KalmanFilter>(m_dt, m_n, m_p, m_m, m_F, m_G, m_H, R, Q);
}

Estimator::Estimator(const Estimator& O)
{
    m_n = O.m_n;
    m_m = O.m_m;
    m_p = O.m_p;
    m_dt = O.m_dt;

    m_F = O.m_F;
    m_G = O.m_G;
    m_H = O.m_H;

    m_R = O.m_R;
    m_Q = O.m_Q;

    m_kf = std::make_unique<KalmanFilter>(m_dt, m_n, m_p, m_m, m_F, m_G, m_H, m_R, m_Q);
}

Estimator::~Estimator()
{
}

bool Estimator::initialize(const Eigen::MatrixXd& z0)
{

    Eigen::MatrixXd x0 = m_H * z0;
    Eigen::MatrixXd M0 = Eigen::MatrixXd::Identity(m_n, m_n);

    return m_kf->initialize(x0, M0);
}

bool Estimator::estimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat)
{
    return m_kf->estimateNextState(z, x_hat);
}

bool Estimator::estimateNextState(const Eigen::MatrixXd& z)
{
    return m_kf->estimateNextState(z);
}

bool Estimator::estimateNextSteadyState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat)
{
    return m_kf->estimateNextSteadyState(z, x_hat);
}

bool Estimator::estimateNextSteadyState(const Eigen::MatrixXd& z)
{
    return m_kf->estimateNextSteadyState(z);
}

void Estimator::getInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P)
{
    return m_kf->getInfo(x_hat, P);
}

void Estimator::getExpetedStateInfo(Eigen::VectorXd& x_hat)
{
    return m_kf->getExpetedStateInfo(x_hat);
}

void Estimator::getCovInfo(Eigen::VectorXd& P)
{
    return m_kf->getCovInfo(P);
}
