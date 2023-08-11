// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MOTOR_ESTIMATION_HPP
#define MOTOR_ESTIMATION_HPP
#include <ControlHelper.hpp>
#include <KalmanFilter.hpp>
#include <memory>

namespace HapticGlove
{
class Estimator;
} // namespace HapticGlove

/**
 * Estimator Class useful to estimating the states of a single joint/axis.
 */
class HapticGlove::Estimator
{
    double m_dt; /// <summary> sampling time
    size_t m_n; /// <summary>  number of states
    size_t m_m; /// <summary>  number of input vector w
    size_t m_p; /// <summary>  number of measures

    CtrlHelper::Eigen_Mat
        m_F; /// <summary>  LTI Continuous system dynamics Matrix Dx(t)= Fx(t)+ Gw(t), size: n*n
    CtrlHelper::Eigen_Mat
        m_G; /// <summary>  LTI Continuous system input Matrix Dx(t)= Fx(t)+ Gw(t), size:  n*m
    CtrlHelper::Eigen_Mat m_H; /// <summary>  Measurement Matrix Z(t)= Hx(t)+ v(t), size: p*n

    std::unique_ptr<KalmanFilter>
        m_kf; /// <summary> vector of kalman filters for each joint estimation

    CtrlHelper::Eigen_Mat m_R; /// <summary>  E[ v(t) v(t)^T ], size:  p*p positive matrix
    CtrlHelper::Eigen_Mat
        m_Q; /// <summary>  E[ (w(t) -w_bar(t)) (w(t) -w_bar(t))^T ], size:  m*m positive matrix

public:
    /**
     * constructor.
     * @param dt sampling time
     * @param R E[ v(t) v(t)^T ],
     * @param Q E[ (w(t) -w_bar(t)) (w(t) -w_bar(t))^T ]
     */
    Estimator(const double& dt, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q);

    /**
     * copy constructor.
     * @param O an estimator constant reference
     */
    Estimator(const Estimator& O);

    /**
     * destructor.
     */
    ~Estimator();

    /**
     * intialize the motor/joint estimator
     * @param  z0 initial measurements
     */
    bool initialize(const Eigen::MatrixXd& z0);

    /**
     * perform the estimatatiom step
     * @param  z measurements
     * @param  x_hat estimated states
     */
    bool estimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    /**
     * perform the estimatatiom step
     * @param  z measurements
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
#endif // MOTOR_ESTIMATION_HPP
