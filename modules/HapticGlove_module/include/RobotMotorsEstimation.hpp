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

    std::vector<std::unique_ptr<MotorEstimation>> m_motorEstimatorVector;
    yarp::sig::Vector m_motorValueMeasured;
    yarp::sig::Vector m_motorValueEstimation;
    yarp::sig::Vector m_motorVelocityEstimation;
    yarp::sig::Vector m_motorAccelerationEstimation;
    size_t m_numerOfMotors;

public:
    RobotMotorsEstimation(const int noMotors);
    bool configure(const yarp::os::Searchable& config, const std::string& name);

    bool initialize(const yarp::sig::Vector& z0);

    bool estimateNextState(const yarp::sig::Vector z, yarp::sig::Vector& x_hat);

    bool  getInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P);
    bool  getInfo(yarp::sig::Vector& x_hat, Eigen::VectorXd& P);
    bool  getInfo(std::vector<double>& x_hat, Eigen::VectorXd& P);


};
#endif // ROBOTMOTORSESTIMATION_HPP
