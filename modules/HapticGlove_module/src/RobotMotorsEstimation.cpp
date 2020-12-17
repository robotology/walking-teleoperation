/**
 * @file RobotMotorsEstimation.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <yarp/os/LogStream.h>
#include <Utils.hpp>

#include <RobotMotorsEstimation.hpp>

RobotMotorsEstimation::RobotMotorsEstimation(const int noMotors){
    m_numerOfMotors=noMotors;
}

bool RobotMotorsEstimation::configure(const yarp::os::Searchable& config, const std::string& name){

    size_t no_states_kf, no_measurement_kf;
    double dt;

    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", dt))
    {
        yError() << "[RobotMotorsEstimation::configure] Unable to find the samplingTime";
        return false;
    }

    no_states_kf=config.check("no_states_kf", yarp::os::Value(3)).asInt();
    no_measurement_kf=config.check("no_measurement_kf", yarp::os::Value(1)).asInt();

    yarp::sig::Vector Q_vector(no_states_kf, 0.0), R_vector(no_measurement_kf, 0.0);
    if (!YarpHelper::getYarpVectorFromSearchable(config, "r_matrix_kf", R_vector))
    {
        yError() << "[RobotController::configure] Initialization failed while reading "
                    "r_matrix_kf.";
        return false;
    }

    if (!YarpHelper::getYarpVectorFromSearchable(config, "q_matrix_kf", Q_vector))
    {
        yError() << "[RobotController::configure] Initialization failed while reading "
                    "q_matrix_kf.";
        return false;
    }

    Eigen::MatrixXd Q= Eigen::MatrixXd::Identity(no_states_kf, no_states_kf);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(no_measurement_kf, no_measurement_kf);
    for (int i = 0; i < no_states_kf; i++)
    {
        Q(i, i) = Q_vector(i);
    }
    for (int i = 0; i < no_measurement_kf; i++)
    {
        R(i, i) = R_vector(i);
    }

    for (int i=0; i<m_numerOfMotors; i++)
    {
        MotorEstimation motorEstimator(dt, R, Q);
        m_motorEstimatorVector.push_back(std::make_unique<MotorEstimation>(dt, R, Q) );

    }

    m_motorValueMeasured.resize(m_numerOfMotors, 0.0);
    m_motorValueEstimation.resize(m_numerOfMotors, 0.0);
    m_motorVelocityEstimation.resize(m_numerOfMotors, 0.0);
    m_motorAccelerationEstimation.resize(m_numerOfMotors, 0.0);
}

bool RobotMotorsEstimation::initialize(const yarp::sig::Vector& z0){

}

bool RobotMotorsEstimation::estimateNextState(const yarp::sig::Vector z, yarp::sig::Vector& x_hat){

}

bool  RobotMotorsEstimation::getInfo(Eigen::VectorXd& x_hat, Eigen::VectorXd& P){
}

bool  RobotMotorsEstimation::getInfo(yarp::sig::Vector& x_hat, Eigen::VectorXd& P){
}

bool  RobotMotorsEstimation::getInfo(std::vector<double>& x_hat, Eigen::VectorXd& P){
}

