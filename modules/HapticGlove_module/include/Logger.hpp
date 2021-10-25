/**
 * @file Logger.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

// std
#include <vector>

// teleoperation
#include <Teleoperation.hpp>

// matlogger
#ifdef ENABLE_LOGGER
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif

class HapticGlove::Teleoperation::Logger
{
public:
    Logger(const Teleoperation& module, const bool isRightHand);
    ~Logger();
    bool openLogger();
    bool updateData();
    bool logData();
    bool closeLogger();

    bool isRightHand;
    std::string handName;
    std::string logPrefix;

    const Teleoperation& teleoperation;

    size_t numOfRobotActuatedAxis;
    size_t numOfRobotActuatedJoints;
    size_t numOfHumanHandFingers;
    size_t numOfHumanHandJoints;

    std::string robotPrefix;
    std::string humanPrefix;
    struct Data
    {
        double time;

        std::vector<double> robotAxisReference;
        std::vector<double> robotAxisFeedback;

        std::vector<double> robotMotorCurrnetReference;
        std::vector<double> robotMotorCurrnetFeedback;

        std::vector<double> robotMotorPWMReference;
        std::vector<double> robotMotorPWMFeedback;

        std::vector<double> robotMotorPidOutputs;

        std::vector<double> robotAxisVelocityFeedback;

        std::vector<double> robotAxisValuesReferenceKF;
        std::vector<double> robotAxisVelocitiesReferenceKF;
        std::vector<double> robotAxisAccelerationReferenceKF;
        Eigen::MatrixXd robotAxisCovReferenceKF;

        std::vector<double> robotAxisValuesFeedbackKF;
        std::vector<double> robotAxisVelocitiesFeedbackKF;
        std::vector<double> robotAxisAccelerationFeedbackKF;
        Eigen::MatrixXd robotAxisCovFeedbackKF;

        std::vector<double> robotAxisValueError; // to check: axis Value Error
        std::vector<double> robotAxisVelocityError; // to check: axis Velocity Error

        std::vector<double> robotJointsReference;
        std::vector<double> robotJointsFeedback;

        std::vector<double> robotJointsExpectedKF;
        std::vector<double> robotJointsFeedbackKF;

        std::vector<double> humanJointValues;
        Eigen::MatrixXd humanFingertipPose;
        std::vector<double> humanForceFeedback;
        std::vector<double> humanVibrotactileFeedback;
        std::vector<double> humanPalmRotation;

    } data;
#ifdef ENABLE_LOGGER
    XBot::MatLogger2::Ptr logger; /**< the pointer to the logger */
    XBot::MatAppender::Ptr appender;
#endif
};

#endif // LOGGER_HPP
