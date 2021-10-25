/**
 * @file Logger.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

// teleoperation
#include <Logger.hpp>
#include <Utils.hpp>

// yarp
#include <yarp/os/LogStream.h>

using namespace HapticGlove;
Teleoperation::Logger::Logger(const Teleoperation& module, const bool isRightHand)
    : teleoperation(module)
{
    this->isRightHand = isRightHand;
    this->handName = this->isRightHand ? "Right" : "Left";
    // Robot hand axis
    this->robotPrefix = "robot" + this->handName + "Hand";
    this->humanPrefix = "human" + this->handName + "Hand";

    this->logPrefix = "Logger::" + this->handName + ":: ";

    this->numOfRobotActuatedAxis
        = teleoperation.m_robotController->controlHelper()->getNumberOfActuatedAxis();
    this->numOfRobotActuatedJoints
        = teleoperation.m_robotController->controlHelper()->getNumberOfActuatedJoints();
    this->numOfHumanHandFingers = teleoperation.m_humanGlove->getNumOfFingers();
    this->numOfHumanHandJoints = teleoperation.m_humanGlove->getNumOfHandJoints();

    // initialize the data structure sizes
    data.time = yarp::os::Time::now();
    data.robotAxisReference.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisFeedback.resize(this->numOfRobotActuatedAxis, 0.0);

    data.robotMotorCurrnetReference.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotMotorCurrnetFeedback.resize(this->numOfRobotActuatedAxis, 0.0);

    data.robotMotorPWMReference.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotMotorPWMFeedback.resize(this->numOfRobotActuatedAxis, 0.0);

    data.robotMotorPidOutputs.resize(this->numOfRobotActuatedAxis, 0.0);

    data.robotAxisVelocityFeedback.resize(this->numOfRobotActuatedAxis, 0.0);

    data.robotAxisValuesReferenceKF.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisVelocitiesReferenceKF.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisAccelerationReferenceKF.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisCovReferenceKF = Eigen::MatrixXd::Zero(this->numOfRobotActuatedAxis, 9);

    data.robotAxisValuesFeedbackKF.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisVelocitiesFeedbackKF.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisAccelerationFeedbackKF.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisCovFeedbackKF = Eigen::MatrixXd::Zero(this->numOfRobotActuatedAxis, 9);

    data.robotAxisValueError.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisVelocityError.resize(this->numOfRobotActuatedAxis, 0.0);

    data.robotJointsReference.resize(this->numOfRobotActuatedJoints, 0.0);
    data.robotJointsFeedback.resize(this->numOfRobotActuatedJoints, 0.0);

    data.robotJointsExpectedKF.resize(this->numOfRobotActuatedJoints, 0.0);
    data.robotJointsFeedbackKF.resize(this->numOfRobotActuatedJoints, 0.0);

    data.humanJointValues.resize(this->numOfHumanHandJoints, 0.0);
    data.humanFingertipPose = Eigen::MatrixXd::Zero(this->numOfHumanHandFingers, 7);
    data.humanForceFeedback.resize(this->numOfHumanHandFingers, 0.0);
    data.humanVibrotactileFeedback.resize(this->numOfHumanHandFingers, 0.0);
    data.humanPalmRotation.resize(4, 0.0); // 4: number of quaternions
}

Teleoperation::Logger::~Logger()
{
}

bool Teleoperation::Logger::openLogger()
{
#ifdef ENABLE_LOGGER
    std::string currentTime = YarpHelper::getTimeDateMatExtension();
    std::string fileName
        = "HapticGloveModule_" + this->handName + "Hand_" + currentTime + "_log.mat";

    yInfo() << "log file name: " << currentTime << fileName;
    this->logger = XBot::MatLogger2::MakeLogger(fileName);
    this->appender = XBot::MatAppender::MakeInstance();
    this->appender->add_logger(this->logger);
    this->appender->start_flush_thread();
    // time
    this->logger->create("time", 1);

    // axis
    this->logger->create(this->robotPrefix + "AxisReference", this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisFeedback", this->numOfRobotActuatedAxis);

    // to check if it is real robot or simulation
    if (this->teleoperation.m_robot == "icub")
    {
        // current
        this->logger->create(this->robotPrefix + "MotorCurrnetReference",
                             this->numOfRobotActuatedAxis);
        this->logger->create(this->robotPrefix + "MotorCurrnetFeedback",
                             this->numOfRobotActuatedAxis);
        // pwm
        this->logger->create(this->robotPrefix + "MotorPWMReference", this->numOfRobotActuatedAxis);
        this->logger->create(this->robotPrefix + "MotorPWMFeedback", this->numOfRobotActuatedAxis);
    }

    // pid
    this->logger->create(this->robotPrefix + "MotorPidOutputs", this->numOfRobotActuatedAxis);

    // velocity feedback
    this->logger->create(this->robotPrefix + "AxisVelocityFeedback", this->numOfRobotActuatedAxis);

    // axis reference KF
    this->logger->create(this->robotPrefix + "AxisValuesReferenceKF", this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisVelocitiesReferenceKF",
                         this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisAccelerationReferenceKF",
                         this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisCovReferenceKF",
                         this->numOfRobotActuatedAxis,
                         9); // states: value, velocity, acceleration --> cov matrix size: 3X3=9

    // axis feedback KF
    this->logger->create(this->robotPrefix + "AxisValuesFeedbackKF", this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisVelocitiesFeedbackKF",
                         this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisAccelerationFeedbackKF",
                         this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisCovFeedbackKF",
                         this->numOfRobotActuatedAxis,
                         9); // states: value, velocity, acceleration --> cov matrix size: 3X3=9

    // robot axis errors
    this->logger->create(this->robotPrefix + "AxisValueError", this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisVelocityError", this->numOfRobotActuatedAxis);

    // robot hand joints
    this->logger->create(this->robotPrefix + "JointsReference", this->numOfRobotActuatedJoints);
    this->logger->create(this->robotPrefix + "JointsFeedback", this->numOfRobotActuatedJoints);
    this->logger->create(this->robotPrefix + "JointsExpectedKF", this->numOfRobotActuatedJoints);
    this->logger->create(this->robotPrefix + "JointsFeedbackKF", this->numOfRobotActuatedJoints);

    // Human data
    this->logger->create(this->humanPrefix + "JointValues", this->numOfHumanHandJoints);
    this->logger->create(this->humanPrefix + "FingertipPose", this->numOfHumanHandFingers, 7);
    this->logger->create(this->humanPrefix + "ForceFeedback", this->numOfHumanHandFingers);
    this->logger->create(this->humanPrefix + "VibrotactileFeedback", this->numOfHumanHandFingers);
    this->logger->create(this->humanPrefix + "PalmRotation", 4);

    // add here a part for adding the name of the robot and human fingers and joints.

    yInfo() << this->logPrefix << "logging is active.";

#else
    yInfo() << "[LoggerImplementation::openLogger] option is not active in CMakeLists.";

#endif

    return true;
}

bool Teleoperation::Logger::updateData()
{
    // initialize the data structure sizes

    data.time = yarp::os::Time::now();

    this->teleoperation.m_robotController->getFingerAxisValueReference(data.robotAxisReference);

    this->teleoperation.m_robotController->getFingerAxisFeedback(data.robotAxisFeedback);

    if (this->teleoperation.m_robot == "icub")

    {

        this->teleoperation.m_robotController->getMotorCurrentReference(
            data.robotMotorCurrnetReference);

        this->teleoperation.m_robotController->getMotorCurrentFeedback(
            data.robotMotorCurrnetFeedback);

        this->teleoperation.m_robotController->getMotorPwmReference(data.robotMotorPWMReference);

        this->teleoperation.m_robotController->getMotorPwmFeedback(data.robotMotorPWMFeedback);
    }

    this->teleoperation.m_robotController->getMotorPidOutputs(data.robotMotorPidOutputs);

    this->teleoperation.m_robotController->getFingerAxisVelocityFeedback(
        data.robotAxisVelocityFeedback);

    this->teleoperation.m_robotController->getEstimatedMotorsState(
        data.robotAxisValuesFeedbackKF,
        data.robotAxisVelocitiesFeedbackKF,
        data.robotAxisAccelerationFeedbackKF,
        data.robotAxisCovFeedbackKF,
        data.robotAxisValuesReferenceKF,
        data.robotAxisVelocitiesReferenceKF,
        data.robotAxisAccelerationReferenceKF,
        data.robotAxisCovReferenceKF);

    for (size_t i = 0; i < this->numOfRobotActuatedAxis; i++)
        data.robotAxisValueError[i] = this->teleoperation.m_robotAxisValueErrors[i];

    for (size_t i = 0; i < this->numOfRobotActuatedAxis; i++)
        data.robotAxisVelocityError[i] = this->teleoperation.m_robotAxisVelocityErrors[i];

    this->teleoperation.m_robotController->getFingerJointReference(data.robotJointsReference);

    this->teleoperation.m_robotController->getFingerJointsFeedback(data.robotJointsFeedback);

    this->teleoperation.m_robotController->getEstimatedJointState(data.robotJointsFeedbackKF,
                                                                  data.robotJointsExpectedKF);

    this->teleoperation.m_humanGlove->getHandJointAngles(data.humanJointValues);

    this->teleoperation.m_humanGlove->getHandJointAngles(data.humanJointValues);

    data.humanFingertipPose = Eigen::MatrixXd::Zero(this->numOfHumanHandFingers, 7); // to implement
    data.humanForceFeedback.resize(this->numOfHumanHandFingers, 0.0); // to implement
    data.humanVibrotactileFeedback.resize(this->numOfHumanHandFingers, 0.0); // to implement

    this->teleoperation.m_humanGlove->getHandPalmRotation(data.humanPalmRotation);

    return true;
}
bool Teleoperation::Logger::logData()
{
#ifdef ENABLE_LOGGER

    if (!this->updateData())
    {
        yError() << this->logPrefix << "cannot update the data.";
    }

    this->logger->add("time", this->data.time);

    this->logger->add(this->robotPrefix + "AxisReference", this->data.robotAxisReference);
    this->logger->add(this->robotPrefix + "AxisFeedback", this->data.robotAxisFeedback);

    // to check if it is real robot or simulation
    if (this->teleoperation.m_robot == "icub")
    {
        this->logger->add(this->robotPrefix + "MotorCurrnetReference",
                          this->data.robotMotorCurrnetReference);
        this->logger->add(this->robotPrefix + "MotorCurrnetFeedback",
                          this->data.robotMotorCurrnetFeedback);

        this->logger->add(this->robotPrefix + "MotorPWMReference",
                          this->data.robotMotorPWMReference);
        this->logger->add(this->robotPrefix + "MotorPWMFeedback", this->data.robotMotorPWMFeedback);
    }

    this->logger->add(this->robotPrefix + "MotorPidOutputs", this->data.robotMotorPidOutputs);

    this->logger->add(this->robotPrefix + "AxisVelocityFeedback",
                      this->data.robotAxisVelocityFeedback);

    this->logger->add(this->robotPrefix + "AxisValuesReferenceKF",
                      this->data.robotAxisValuesReferenceKF);
    this->logger->add(this->robotPrefix + "AxisVelocitiesReferenceKF",
                      this->data.robotAxisVelocitiesReferenceKF);
    this->logger->add(this->robotPrefix + "AxisAccelerationReferenceKF",
                      this->data.robotAxisAccelerationReferenceKF);
    this->logger->add(this->robotPrefix + "AxisCovReferenceKF", this->data.robotAxisCovReferenceKF);

    this->logger->add(this->robotPrefix + "AxisValuesFeedbackKF",
                      this->data.robotAxisValuesFeedbackKF);
    this->logger->add(this->robotPrefix + "AxisVelocitiesFeedbackKF",
                      this->data.robotAxisVelocitiesFeedbackKF);
    this->logger->add(this->robotPrefix + "AxisAccelerationFeedbackKF",
                      this->data.robotAxisAccelerationFeedbackKF);
    this->logger->add(this->robotPrefix + "AxisCovFeedbackKF",
                      this->data.robotAxisCovFeedbackKF); // states: value, velocity, acceleration
    // --> cov matrix size: 3X3=9

    // robot axis errors
    this->logger->add(this->robotPrefix + "AxisValueError", this->data.robotAxisValueError);
    this->logger->add(this->robotPrefix + "AxisVelocityError", this->data.robotAxisVelocityError);

    // Robot hand joints
    this->logger->add(this->robotPrefix + "JointsReference", this->data.robotJointsReference);
    this->logger->add(this->robotPrefix + "JointsFeedback", this->data.robotJointsFeedback);
    this->logger->add(this->robotPrefix + "JointsExpectedKF", this->data.robotJointsExpectedKF);
    this->logger->add(this->robotPrefix + "JointsFeedbackKF", this->data.robotJointsFeedbackKF);

    // Human info comming from Glove
    this->logger->add(this->humanPrefix + "JointValues", this->data.humanJointValues);
    this->logger->add(this->humanPrefix + "FingertipPose", this->data.humanFingertipPose);
    this->logger->add(this->humanPrefix + "ForceFeedback", this->data.humanForceFeedback);
    this->logger->add(this->humanPrefix + "VibrotactileFeedback",
                      this->data.humanVibrotactileFeedback);
    this->logger->add(this->humanPrefix + "PalmRotation", data.humanPalmRotation);
#endif

    return true;
}

bool Teleoperation::Logger::closeLogger()
{

#ifdef ENABLE_LOGGER
    this->logger->flush_available_data();
#endif
    yInfo() << this->logPrefix << "logger is closing.";
    return true;
}
