/**
 * @file MotorEstimation.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef MOTORESTIMATION_HPP
#define MOTORESTIMATION_HPP
#include <Eigen/Dense>
#include <KalmanFilter.hpp>
#include <memory>

namespace HapticGlove
{
class Estimator;
}

class HapticGlove::Estimator
{
public:
    double m_dt; /// <summary> sampling time
    size_t m_n; /// <summary>  number of states
    size_t m_m; /// <summary>  number of input vector w
    size_t m_p; /// <summary>  number of measures

    Eigen_Mat
        m_F; /// <summary>  LTI Continuous system dynamics Matrix Dx(t)= Fx(t)+ Gw(t), size: n*n
    Eigen_Mat m_G; /// <summary>  LTI Continuous system input Matrix Dx(t)= Fx(t)+ Gw(t), size:  n*m
    Eigen_Mat m_H; /// <summary>  Measurement Matrix Z(t)= Hx(t)+ v(t), size: p*n

    std::unique_ptr<KalmanFilter>
        m_kf; /// <summary> vector of kalman filters for each joint estimation

    Eigen_Mat m_R; /// <summary>  E[ v(t) v(t)^T ], size:  p*p positive matrix
    Eigen_Mat
        m_Q; /// <summary>  E[ (w(t) -w_bar(t)) (w(t) -w_bar(t))^T ], size:  m*m positive matrix

    Estimator(const double& dt, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q);

    Estimator(const Estimator& O);

    bool Initialize(const Eigen::MatrixXd& z0);

    bool EstimateNextState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    bool EstimateNextSteadyState(const Eigen::MatrixXd& z, Eigen::MatrixXd& x_hat);

    void GetInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P);
};
#endif // MOTORESTIMATION_HPP
