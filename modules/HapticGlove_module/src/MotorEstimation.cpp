/**
 * @file MotorEstimation.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */


#include <MotorEstimation.hpp>
#include <iostream>
Estimator::Estimator(const double& dt, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q){

    m_n = 3;
    m_m = 3;
    m_p = 1;

    m_dt= dt;

    m_F=Eigen::MatrixXd::Zero(m_n,m_n);
    m_F(0,1)=1.0;
    m_F(1,2)=1.0;
    m_G=Eigen::MatrixXd::Identity(m_n,m_m);
    m_H=Eigen::MatrixXd::Zero(m_p,m_n);
    m_H(0,0)=1.0;

    m_R=R;
    m_Q=Q;

    m_kf= std::make_unique<KalmanFilter>(m_dt, m_n, m_p, m_m, m_F, m_G, m_H, R, Q);
}

Estimator::Estimator(const Estimator& O){
    m_n = O.m_n;
    m_m = O.m_m;
    m_p = O.m_p;
    m_dt=O.m_dt;

    m_F=O.m_F;
    m_G=O.m_G;
    m_H=O.m_H;

    m_R=O.m_R;
    m_Q=O.m_Q;

    m_kf=std::make_unique<KalmanFilter>(m_dt, m_n, m_p, m_m, m_F, m_G, m_H, m_R, m_Q);
}

bool Estimator::Initialize(const Eigen::MatrixXd& z0){

    Eigen::MatrixXd x0= m_H * z0;
    Eigen::MatrixXd  M0=Eigen::MatrixXd::Identity(m_n,m_n);

    return m_kf->Initialize(x0,M0);
}

bool Estimator::EstimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat){

    return m_kf->EstimateNextState(z, x_hat);
}

bool Estimator::EstimateNextSteadyState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat)
{
    return m_kf->EstimateNextSteadyState(z, x_hat);

}

bool  Estimator::GetInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P){

    return m_kf->GetInfo( x_hat, P);
}
