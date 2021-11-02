/**
 * @file KalmanFilter.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <KalmanFilter.hpp>
#include <iostream>
using namespace HapticGlove;
KalmanFilter::KalmanFilter(const double dt,
                           const size_t n,
                           const size_t p,
                           const size_t m,
                           const Eigen::MatrixXd& F,
                           const Eigen::MatrixXd& G,
                           const Eigen::MatrixXd& H,
                           const Eigen::MatrixXd& R,
                           const Eigen::MatrixXd& Q)
    : m_dt(dt)
    , m_n(n)
    , m_p(p)
    , m_m(m)
    , m_F(F)
    , m_G(G)
    , m_H(H)
    , m_R(R)
    , m_Q(Q)
{
    m_logPrefix = "KalmanFilter:: ";
    std::cout << m_logPrefix << "constructing Kalman filter.\n";

    m_Phi = Eigen::MatrixXd::Identity(n, n) + m_F * m_dt;
    m_Gamma = m_G * m_dt;
    m_Ht_Rinv = m_H.transpose() * m_R.inverse();
    m_Ht_Rinv_H = m_H.transpose() * m_R.inverse() * m_H;
    Gamma_Q_GammaT = m_Gamma * m_Q * m_Gamma.transpose();

    m_w_bar = Eigen::MatrixXd::Zero(m_m, 1);

    std::cout << m_logPrefix << "constructed.\n";
}

KalmanFilter::~KalmanFilter()
{
}

bool KalmanFilter::initialize(const Eigen::MatrixXd& x0, const Eigen::MatrixXd& M0)
{
    m_x_bar = x0;
    m_M = M0;

    m_P = (m_M.inverse() + m_Ht_Rinv_H).inverse();
    m_K = m_P * m_Ht_Rinv;
    m_M = m_Phi * m_P * m_Phi.transpose() + m_Gamma * m_Q * m_Gamma.transpose();

    return true;
}

bool KalmanFilter::setNewMeasurements(const Eigen::MatrixXd& z)
{
    m_z = z;
    return true;
}

bool KalmanFilter::estimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat)
{
    /*
     * J= 1/2 [(x-x_bar) M^(-1)(x-x_bar) + (z-Hx) R^(-1)(z-Hx)]
     */

    m_z = z;

    m_P = (m_M.inverse() + m_Ht_Rinv_H).inverse();
    m_K = m_P * m_Ht_Rinv;
    m_x_hat = m_x_bar + m_K * (m_z - m_H * m_x_bar);
    m_x_bar = m_Phi * m_x_hat + m_Gamma * m_w_bar;
    m_M = m_Phi * m_P * m_Phi.transpose() + m_Gamma * m_Q * m_Gamma.transpose();

    x_hat = m_x_hat;

    return true;
}

bool KalmanFilter::estimateNextState(const Eigen::MatrixXd& z)
{
    /*
     * J= 1/2 [(x-x_bar) M^(-1)(x-x_bar) + (z-Hx) R^(-1)(z-Hx)]
     */

    m_z = z;

    m_P = (m_M.inverse() + m_Ht_Rinv_H).inverse();
    m_K = m_P * m_Ht_Rinv;
    m_x_hat = m_x_bar + m_K * (m_z - m_H * m_x_bar);
    m_x_bar = m_Phi * m_x_hat + m_Gamma * m_w_bar;
    m_M = m_Phi * m_P * m_Phi.transpose() + m_Gamma * m_Q * m_Gamma.transpose();

    std::cout << "m_z : " << m_z.transpose() << "\n";
    std::cout << "m_K : " << m_K << "\n";
    std::cout << "m_x_hat transpose: " << m_x_hat.transpose() << "\n";
    std::cout << "m_x_bar transpose: " << m_x_bar.transpose() << "\n";
    std::cout << "m_P: " << m_P << "\n";
    std::cout << "m_M: " << m_M << "\n";

    return true;
}

bool KalmanFilter::estimateNextSteadyState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat)
{

    /*
     * J= 1/2 [(x-x_bar) M^(-1)(x-x_bar) + (z-Hx) R^(-1)(z-Hx)]
     */

    m_z = z;

    m_x_hat = m_x_bar + m_K * (m_z - m_H * m_x_bar);
    m_x_bar = m_Phi * m_x_hat + m_Gamma * m_w_bar;

    x_hat = m_x_hat;

    return true;
}

bool KalmanFilter::estimateNextSteadyState(const Eigen::MatrixXd& z)
{
    /*
     * J= 1/2 [(x-x_bar) M^(-1)(x-x_bar) + (z-Hx) R^(-1)(z-Hx)]
     */

    m_z = z;

    m_x_hat = m_x_bar + m_K * (m_z - m_H * m_x_bar);
    m_x_bar = m_Phi * m_x_hat + m_Gamma * m_w_bar;

    return true;
}

void KalmanFilter::getInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P)
{
    this->getExpetedStateInfo(x_hat);

    this->getCovInfo(P);
}

void KalmanFilter::getExpetedStateInfo(Eigen::VectorXd& x_hat)
{
    x_hat = Eigen::Map<Eigen::VectorXd>(m_x_hat.data(), m_x_hat.cols() * m_x_hat.rows());
}

void KalmanFilter::getCovInfo(Eigen::VectorXd& P)
{
    P = Eigen::Map<Eigen::VectorXd>(m_P.data(), m_P.cols() * m_P.rows());
}
