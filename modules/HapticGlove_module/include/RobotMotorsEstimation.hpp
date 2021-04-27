/**
 * @file RobotMotorsEstimation.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */


#ifndef ROBOTOTORSESTIMATION_HPP
#define ROBOTMOTORSESTIMATION_HPP
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <MotorEstimation.hpp>

#include <yarp/os/Property.h>


typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Eigen_Mat;

class RobotMotorsEstimation{
private:

    std::vector<MotorEstimation> m_motorEstimatorVector;
    yarp::sig::Vector m_motorValueMeasured;
    yarp::sig::Vector m_motorValueEstimation;
    yarp::sig::Vector m_motorVelocityEstimation;
    yarp::sig::Vector m_motorAccelerationEstimation;
    size_t m_numerOfMotors;

    Eigen::MatrixXd z_mat;

    bool m_isInitialized;

public:
    RobotMotorsEstimation(const int noMotors);
    bool configure(const yarp::os::Searchable& config, const std::string& name);

    bool initialize(const yarp::sig::Vector& z0);

    bool estimateNextState(const yarp::sig::Vector z, yarp::sig::Vector& x_hat);

    bool estimateNextState(const yarp::sig::Vector z );

    bool  getInfo(Eigen::VectorXd& estimatedMotorValue, Eigen::VectorXd& estimatedMotorVelocity, Eigen::VectorXd& estimatedMotorAcceleration, Eigen::MatrixXd& P);
    bool  getInfo(yarp::sig::Vector& estimatedMotorValue, yarp::sig::Vector& estimatedMotorVelocity, yarp::sig::Vector& estimatedMotorAcceleration, Eigen::MatrixXd& P);
    bool  getInfo(std::vector<double>& estimatedMotorValue, std::vector<double>&  estimatedMotorVelocity, std::vector<double>& estimatedMotorAcceleration, Eigen::MatrixXd& P);

    bool isInitialized();

};
#endif // ROBOTMOTORSESTIMATION_HPP
