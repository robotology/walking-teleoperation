/**
 * @file KalmanFilter.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */


/*
 * Implementation based on Book:
 * Applied Optimal Control: Optimization, Estimation, and Control
 * ByArthur E. Bryson, Yu-Chi Ho
 * Chapter 12|42 pages: Optimal filtering and prediction
*/

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Eigen_Mat;

class KalmanFilter{
private:



    size_t m_n; /**< size state vector (x)*/
    size_t m_m; /**< size of w vector*/
    size_t m_p; /**< Size of measure vecotr(z)*/
    double m_dt;/**< sampling time*/

    Eigen_Mat m_F; /**< LTI Continuous system dynamics Matrix Dx(t)= Fx(t)+ Gw(t), size: n*n */
    Eigen_Mat m_G; /**< LTI Continuous system input Matrix Dx(t)= Fx(t)+ Gw(t), size:  n*m */
    Eigen_Mat m_H; /**< Measurement Matrix Z(t)= Hx(t)+ v(t), size: p*n */

    Eigen_Mat m_Phi; /**< LTI Discrete system dynamics Matrix x(i+1)= Phi X(i)+ Gamma w(i), size: n*n */
    Eigen_Mat m_Gamma; /**< LTI Discrete system input Matrix x(i+1)= Phi X(i)+ Gamma w(i), size: n*m */

    Eigen_Mat m_M; /**< E[ (x(t)- x_bar(t))(x(t)- x_bar(t))^T ], size:  n*n positive matrix */
    Eigen_Mat m_R; /**< E[ v(t) v(t)^T ], size:  p*p positive matrix */
    Eigen_Mat m_Q;  /**< E[ (w(t) -w_bar(t)) (w(t) -w_bar(t))^T ], size:  m*m positive matrix */

    Eigen_Mat m_P; /**< P_inv= M_inv + H^T R_inv H, size: n*n */
    Eigen_Mat m_K; /**< P H^T R_inv, size: n*p */

    Eigen_Mat m_x_bar; /**< state estimation before using the measurements, size: n*1 */
    Eigen_Mat m_x_hat; /**< E[x(t)], size: n*1  */
    Eigen_Mat m_w_bar; /**< E[w(t)], size: m*1 */
    Eigen_Mat m_z; /**< z(t) measurement vector, size: p*1 */

    Eigen_Mat m_Ht_Rinv;/**< H^T R^(-1), size: n*p */
    Eigen_Mat m_Ht_Rinv_H;/**< H^T R^(-1), size: n*n*/
    Eigen_Mat Gamma_Q_GammaT;/**< Gamma Q Gamma^T, size: n*n */

public:

    KalmanFilter(const double dt,
                 const size_t n,
                 const size_t p,
                 const size_t m,
                 const Eigen::MatrixXd& F,
                 const Eigen::MatrixXd& G,
                 const Eigen::MatrixXd& H,
                 const Eigen::MatrixXd& R,
                 const Eigen::MatrixXd& Q);
    ~KalmanFilter();

    bool Initialize(const Eigen::MatrixXd& x0, const Eigen::MatrixXd& M0);

    bool  SetNewMeasure(const Eigen::MatrixXd& z);

    bool  EstimateNextState(Eigen::MatrixXd& x_hat);

    bool EstimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    bool EstimateNextSteadyState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    bool  GetInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P);

};
#endif // KALMANFILTER_H
