/**
 * @file MotorEstimation.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */


#include <MotorEstimation.hpp>

MotorEstimation::MotorEstimation(const double& dt, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q){

    m_n = 3;
    m_m = 3;
    m_p = 1;

    m_F=Eigen::MatrixXd::Zero(m_n,m_n);
    m_F(0,1)=1.0;
    m_F(1,2)=1.0;
    m_G=Eigen::MatrixXd::Identity(m_n,m_m);
    m_H=Eigen::MatrixXd::Zero(m_p,m_n);
    m_H(0,0)=1.0;


    m_kf= std::make_unique<KalmanFilter>(dt, m_n, m_p, m_m, m_F, m_G, m_H, R, Q);

}
MotorEstimation::~MotorEstimation(){
    m_kf->~KalmanFilter();
}
bool MotorEstimation::Initialize(const Eigen::MatrixXd& z0){

    Eigen::MatrixXd x0= m_H * z0;
    Eigen::MatrixXd  M0=Eigen::MatrixXd::Identity(m_n,m_n);

    return m_kf->Initialize(x0,M0);
}

bool MotorEstimation::EstimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat){

    return m_kf->EstimateNextState(z, x_hat);
}

bool  MotorEstimation::GetInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P){

    return m_kf->GetInfo( x_hat, P);
}
