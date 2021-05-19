/**
 * @file MotorEstimation.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */


#ifndef MOTORESTIMATION_HPP
#define MOTORESTIMATION_HPP
#include <memory>
#include <Eigen/Dense>
#include <KalmanFilter.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Eigen_Mat;

class MotorEstimation{
public:
    double m_dt;
    size_t m_n ; // number of states
    size_t m_m ; // number of input vector w
    size_t m_p ; // number of measures


    Eigen_Mat m_F; /**< LTI Continuous system dynamics Matrix Dx(t)= Fx(t)+ Gw(t), size: n*n */
    Eigen_Mat m_G; /**< LTI Continuous system input Matrix Dx(t)= Fx(t)+ Gw(t), size:  n*m */
    Eigen_Mat m_H; /**< Measurement Matrix Z(t)= Hx(t)+ v(t), size: p*n */


    std::unique_ptr<KalmanFilter> m_kf;


    Eigen_Mat m_R; /**< E[ v(t) v(t)^T ], size:  p*p positive matrix */
    Eigen_Mat m_Q;  /**< E[ (w(t) -w_bar(t)) (w(t) -w_bar(t))^T ], size:  m*m positive matrix */


    MotorEstimation(const double& dt, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q);
    MotorEstimation(const MotorEstimation& O);

    bool Initialize(const Eigen::MatrixXd& z0);

    bool EstimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    bool EstimateNextSteadyState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);


    bool  GetInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P);

};
#endif // MOTORESTIMATION_HPP
