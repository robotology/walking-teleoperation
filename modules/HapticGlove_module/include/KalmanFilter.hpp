// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/*
 * Implementation based on Book:
 * Applied Optimal Control: Optimization, Estimation, and Control
 * ByArthur E. Bryson, Yu-Chi Ho
 * Chapter 12|42 pages: Optimal filtering and prediction
 */

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <ControlHelper.hpp>

namespace HapticGlove
{
class KalmanFilter;
} // namespace HapticGlove

/**
 * KalmanFilter Class is a Kalman filter implementation.
 */
class HapticGlove::KalmanFilter
{
private:
    std::string m_logPrefix;
    size_t m_n; /// <summary>  size state vector (x)
    size_t m_m; /// <summary>  size of w vector
    size_t m_p; /// <summary>  Size of measure vector(z)
    double m_dt; /// <summary>  sampling time

    CtrlHelper::Eigen_Mat
        m_F; /// <summary>  LTI Continuous system dynamics Matrix Dx(t)= Fx(t)+ Gw(t), size: n*n
    CtrlHelper::Eigen_Mat
        m_G; /// <summary>  LTI Continuous system input Matrix Dx(t)= Fx(t)+ Gw(t), size:  n*m
    CtrlHelper::Eigen_Mat m_H; /// <summary>  Measurement Matrix Z(t)= Hx(t)+ v(t), size: p*n

    CtrlHelper::Eigen_Mat m_Phi; /// <summary>  LTI Discrete system dynamics Matrix x(i+1)= Phi
                                 /// X(i)+ Gamma w(i), size: n*n
    CtrlHelper::Eigen_Mat m_Gamma; /// <summary>  LTI Discrete system input Matrix x(i+1)= Phi X(i)+
                                   /// Gamma w(i), size: n*m

    CtrlHelper::Eigen_Mat
        m_M; /// <summary>  E[ (x(t)- x_bar(t))(x(t)- x_bar(t))^T ], size:  n*n positive matrix
    CtrlHelper::Eigen_Mat m_R; /// <summary>  E[ v(t) v(t)^T ], size:  p*p positive matrix */
    CtrlHelper::Eigen_Mat
        m_Q; /// <summary>  E[ (w(t) -w_bar(t)) (w(t) -w_bar(t))^T ], size:  m*m positive matrix

    CtrlHelper::Eigen_Mat m_P; /// <summary>  P_inv= M_inv + H^T R_inv H, size: n*n
    CtrlHelper::Eigen_Mat m_K; /// <summary>  P H^T R_inv, size: n*p

    CtrlHelper::Eigen_Mat
        m_x_bar; /// <summary>  state estimation before using the measurements, size: n*1
    CtrlHelper::Eigen_Mat m_x_hat; /// <summary>  E[x(t)], size: n*1
    CtrlHelper::Eigen_Mat m_w_bar; /// <summary>  E[w(t)], size: m*1
    CtrlHelper::Eigen_Mat m_z; /// <summary>  z(t) measurement vector, size: p*1

    CtrlHelper::Eigen_Mat m_Ht_Rinv; /// <summary>  H^T R^(-1), size: n*p
    CtrlHelper::Eigen_Mat m_Ht_Rinv_H; /// <summary>  H^T R^(-1), size: n*n
    CtrlHelper::Eigen_Mat Gamma_Q_GammaT; /// <summary>  Gamma Q Gamma^T, size: n*n

public:
    /**
     * constructor.
     * @param dt sampling time
     * @param n state vector size
     * @param p measurement vector size
     * @param m size of w vector
     * @param F LTI Continuous system dynamics Matrix
     * @param G LTI Continuous system input Matrix
     * @param H Measurement Matrix
     * @param R E[ v(t) v(t)^T ],
     * @param Q E[ (w(t) -w_bar(t)) (w(t) -w_bar(t))^T ]
     */
    KalmanFilter(const double dt,
                 const size_t n,
                 const size_t p,
                 const size_t m,
                 const Eigen::MatrixXd& F,
                 const Eigen::MatrixXd& G,
                 const Eigen::MatrixXd& H,
                 const Eigen::MatrixXd& R,
                 const Eigen::MatrixXd& Q);

    /**
     * destructor.
     */
    ~KalmanFilter();

    /**
     * intialize the Kalman filter
     * @param  x0 initial state estimation before using the measurements
     * @param  M0 initial covariance matrix related to < E[ (x(t)- x_bar(t))(x(t)- x_bar(t))^T ]
     */
    bool initialize(const Eigen::MatrixXd& x0, const Eigen::MatrixXd& M0);

    /**
     * set the new measurements.
     * @param z new measurement vector
     */
    bool setNewMeasurements(const Eigen::MatrixXd& z);

    /**
     * set the measurement vector and perform an estimation step and get the updated expected state.
     * @param z new measurement vector
     * @param x_hat updated expected state
     */
    bool estimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    /**
     * set the measurement vector and perform an estimation step and get the updated expected state.
     * @param z new measurement vector
     */
    bool estimateNextState(const Eigen::MatrixXd& z);

    /**
     * set the measurement vector and perform an estimation step assuming stochastic steady state
     * system and get the updated expected state.
     * @param z new measurement vector
     * @param x_hat updated expected state
     */
    bool estimateNextSteadyState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    /**
     * set the measurement vector and perform an estimation step assuming stochastic steady state
     * system and get the updated expected state.
     * @param z new measurement vector
     */
    bool estimateNextSteadyState(const Eigen::MatrixXd& z);

    /**
     * get the estimation results
     * @param x_hat expected state results
     * @param P covariance of the estimated state
     */
    void getInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P);

    /**
     * get the estimation results
     * @param x_hat expected state results
     */
    void getExpetedStateInfo(Eigen::VectorXd& x_hat);

    /**
     * get the estimation results
     * @param P covariance of the estimated state
     */
    void getCovInfo(Eigen::VectorXd& P);
};
#endif // KALMAN_FILTER_HPP
