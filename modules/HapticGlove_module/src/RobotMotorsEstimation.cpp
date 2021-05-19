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
    m_isInitialized=false;
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
        std::cout<<"MotorEstimation initialization ...  \n";
        MotorEstimation motorEstimator(dt, R, Q);
        m_motorEstimatorVector.push_back(motorEstimator);
    }
    std::cout<<"All Motor Estimation are initialized. \n";


    m_motorValueMeasured.resize(m_numerOfMotors*no_measurement_kf, 0.0);
    m_motorValueEstimation.resize(m_numerOfMotors, 0.0);
    m_motorVelocityEstimation.resize(m_numerOfMotors, 0.0);
    m_motorAccelerationEstimation.resize(m_numerOfMotors, 0.0);

    z_mat=Eigen::MatrixXd::Zero(no_measurement_kf,1);
    return true;
}

bool RobotMotorsEstimation::initialize(const yarp::sig::Vector& z0){

    m_motorValueMeasured= z0;

    for(size_t i=0; i<m_numerOfMotors;i++)
    {
        z_mat(0,0)=z0(i);
        m_motorEstimatorVector[i].Initialize(z_mat);
    }
    m_isInitialized=true;
    return true;
}

bool RobotMotorsEstimation::estimateNextState(const yarp::sig::Vector z, yarp::sig::Vector& x_hat){

    Eigen::MatrixXd x_hat_mat;
    for(size_t i=0; i<m_numerOfMotors;i++)
    {
        z_mat(0,0)=z(i);
        m_motorEstimatorVector[i].EstimateNextState(z_mat, x_hat_mat);

//        m_motorValueEstimation[i]=x_hat[0];
//        m_motorVelocityEstimation[i]=x_hat[1];
//        m_motorAccelerationEstimation[i]=x_hat[2];
    }
    return true;
}

bool RobotMotorsEstimation::estimateNextState(const yarp::sig::Vector z ){

    Eigen::MatrixXd x_hat_mat;
    for(size_t i=0; i<m_numerOfMotors;i++)
    {
        z_mat(0,0)=z(i);
        m_motorEstimatorVector[i].EstimateNextState(z_mat, x_hat_mat);

//        m_motorValueEstimation[i]=x_hat[0];
//        m_motorVelocityEstimation[i]=x_hat[1];
//        m_motorAccelerationEstimation[i]=x_hat[2];
    }
    return true;
}

bool RobotMotorsEstimation::estimateNextSteadyState(const yarp::sig::Vector z )
{

    Eigen::MatrixXd x_hat_mat;
    for(size_t i=0; i<m_numerOfMotors;i++)
    {
        z_mat(0,0)=z(i);
        m_motorEstimatorVector[i].EstimateNextSteadyState(z_mat, x_hat_mat);
    }
    return true;
}


bool  RobotMotorsEstimation::getInfo(Eigen::VectorXd& estimatedMotorValue, Eigen::VectorXd& estimatedMotorVelocity, Eigen::VectorXd& estimatedMotorAcceleration, Eigen::MatrixXd& P){

    estimatedMotorValue.resize(m_numerOfMotors, 0.0);
    estimatedMotorVelocity.resize(m_numerOfMotors, 0.0);
    estimatedMotorAcceleration.resize(m_numerOfMotors, 0.0);
    P.resize(m_numerOfMotors, 9);

    for(size_t i=0; i<m_numerOfMotors;i++)
    {
        Eigen::VectorXd x_hat;
        Eigen::VectorXd P_tmp;
        m_motorEstimatorVector[i].GetInfo(x_hat, P_tmp);

        estimatedMotorValue[i]=x_hat[0];
        estimatedMotorVelocity[i]=x_hat[1];
        estimatedMotorAcceleration[i]=x_hat[2];
        P.row(i)= P_tmp;
    }

return true;
}

bool  RobotMotorsEstimation::getInfo(yarp::sig::Vector& estimatedMotorValue, yarp::sig::Vector& estimatedMotorVelocity, yarp::sig::Vector& estimatedMotorAcceleration, Eigen::MatrixXd& P){

    estimatedMotorValue.resize(m_numerOfMotors, 0.0);
    estimatedMotorVelocity.resize(m_numerOfMotors, 0.0);
    estimatedMotorAcceleration.resize(m_numerOfMotors, 0.0);
    P.resize(m_numerOfMotors, 9);

    for(size_t i=0; i<m_numerOfMotors;i++)
    {
        Eigen::VectorXd x_hat;
        Eigen::VectorXd P_tmp;
        m_motorEstimatorVector[i].GetInfo(x_hat, P_tmp);

        estimatedMotorValue(i)=x_hat[0];
        estimatedMotorVelocity(i)=x_hat[1];
        estimatedMotorAcceleration(i)=x_hat[2];
        P.row(i)= P_tmp;

    }
return true;
}

bool  RobotMotorsEstimation::getInfo(std::vector<double>& estimatedMotorValue, std::vector<double>&  estimatedMotorVelocity, std::vector<double>& estimatedMotorAcceleration, Eigen::MatrixXd& P){

    estimatedMotorValue.clear();
    estimatedMotorVelocity.clear();
    estimatedMotorAcceleration.clear();

    estimatedMotorValue.resize(m_numerOfMotors, 0.0);
    estimatedMotorVelocity.resize(m_numerOfMotors, 0.0);
    estimatedMotorAcceleration.resize(m_numerOfMotors, 0.0);
    P.resize(m_numerOfMotors, 9);
    P.setZero(m_numerOfMotors, 9);

    for(size_t i=0; i<m_numerOfMotors;i++)
    {
        Eigen::VectorXd x_hat;
        Eigen::VectorXd P_tmp;
        m_motorEstimatorVector[i].GetInfo(x_hat, P_tmp);

        estimatedMotorValue[i]=x_hat[0];
        estimatedMotorVelocity[i]=x_hat[1];
        estimatedMotorAcceleration[i]=x_hat[2];
        P.row(i)= P_tmp;
    }
    return true;
}
bool RobotMotorsEstimation::isInitialized(){

    return m_isInitialized;
}
