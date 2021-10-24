/**
 * @file Logger.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <Logger.hpp>

// yarp
#include <yarp/os/LogStream.h>

HapticGloveModule::Logger::Logger(const HapticGloveModule& module, const bool isRightHand)
    : hapticModule(module)
{
    this->isRightHand = isRightHand;
    this->handName = this->isRightHand ? "Right" : "Left";
    // Robot hand axis
    this->robotPrefix = "robot" + this->handName + "Hand";
    this->humanPrefix = "human" + this->handName + "Hand";

    // fix this if condition later
    if (this->isRightHand)
    {
        this->numOfRobotActuatedAxis
            = hapticModule.m_robotRightHand->controlHelper()->getNumberOfActuatedAxis();
        this->numOfRobotActuatedJoints
            = hapticModule.m_robotRightHand->controlHelper()->getNumberOfActuatedJoints();
        this->numOfHumanHandFingers = hapticModule.m_gloveRightHand->getNumOfFingers();
        this->numOfHumanHandJoints = hapticModule.m_gloveRightHand->getNumOfHandJoints();
    } else
    {
        this->numOfRobotActuatedAxis
            = hapticModule.m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis();
        this->numOfRobotActuatedJoints
            = hapticModule.m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints();
        this->numOfHumanHandFingers = hapticModule.m_gloveLeftHand->getNumOfFingers();
        this->numOfHumanHandJoints = hapticModule.m_gloveLeftHand->getNumOfHandJoints();
    }

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

    data.robotAxisError.resize(this->numOfRobotActuatedAxis, 0.0);
    data.robotAxisErrorSmoothed.resize(this->numOfRobotActuatedAxis, 0.0);

    data.robotJointsReference.resize(this->numOfRobotActuatedJoints, 0.0);
    data.robotJointsFeedback.resize(this->numOfRobotActuatedJoints, 0.0);

    data.robotJointsExpectedKF.resize(this->numOfRobotActuatedJoints, 0.0);
    data.robotJointsFeedbackKF.resize(this->numOfRobotActuatedJoints, 0.0);

    data.humanJointValues.resize(this->numOfHumanHandJoints, 0.0);
    data.humanFingertipPose = Eigen::MatrixXd::Zero(this->numOfHumanHandFingers, 7);
    data.humanForceFeedback.resize(this->numOfHumanHandFingers, 0.0);
    data.humanVibrotactileFeedback.resize(this->numOfHumanHandFingers, 0.0);
    data.humanPalmRotation.resize(this->numOfHumanHandFingers, 0.0);
}

HapticGloveModule::Logger::~Logger()
{
}

bool HapticGloveModule::Logger::openLogger()
{
#ifdef ENABLE_LOGGER
    std::string currentTime = getTimeDateMatExtension();
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
    if (this->hapticModule.m_robot == "icub")
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
    this->logger->create(this->robotPrefix + "AxisError", this->numOfRobotActuatedAxis);
    this->logger->create(this->robotPrefix + "AxisErrorSmoothed", this->numOfRobotActuatedAxis);

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

    yInfo() << "[LoggerImplementation::openLogger] Logging is active.";

#else
    yInfo() << "[LoggerImplementation::openLogger] option is not active in CMakeLists.";

#endif

    return true;
}

bool HapticGloveModule::Logger::updateData()
{
    // initialize the data structure sizes

    data.time = yarp::os::Time::now();

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getFingerAxisValueReference(data.robotAxisReference);
    else
        this->hapticModule.m_robotLeftHand->getFingerAxisValueReference(data.robotAxisReference);

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getFingerAxisFeedback(data.robotAxisFeedback);
    else
        this->hapticModule.m_robotLeftHand->getFingerAxisFeedback(data.robotAxisFeedback);

    if (this->hapticModule.m_robot == "icub")

    {
        if (this->isRightHand)
            this->hapticModule.m_robotRightHand->getMotorCurrentReference(
                data.robotMotorCurrnetReference);
        else
            this->hapticModule.m_robotLeftHand->getMotorCurrentReference(
                data.robotMotorCurrnetReference);

        if (this->isRightHand)
            this->hapticModule.m_robotRightHand->getMotorCurrentFeedback(
                data.robotMotorCurrnetFeedback);
        else
            this->hapticModule.m_robotLeftHand->getMotorCurrentFeedback(
                data.robotMotorCurrnetFeedback);

        if (this->isRightHand)
            this->hapticModule.m_robotRightHand->getMotorPwmReference(data.robotMotorPWMReference);
        else
            this->hapticModule.m_robotLeftHand->getMotorPwmReference(data.robotMotorPWMReference);

        if (this->isRightHand)
            this->hapticModule.m_robotRightHand->getMotorPwmFeedback(data.robotMotorPWMFeedback);
        else
            this->hapticModule.m_robotLeftHand->getMotorPwmFeedback(data.robotMotorPWMFeedback);
    }

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getMotorPidOutputs(data.robotMotorPidOutputs);
    else
        this->hapticModule.m_robotLeftHand->getMotorPidOutputs(data.robotMotorPidOutputs);

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getFingerAxisVelocityFeedback(
            data.robotAxisVelocityFeedback);
    else
        this->hapticModule.m_robotLeftHand->getFingerAxisVelocityFeedback(
            data.robotAxisVelocityFeedback);

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getEstimatedMotorsState(
            data.robotAxisValuesFeedbackKF,
            data.robotAxisVelocitiesFeedbackKF,
            data.robotAxisAccelerationFeedbackKF,
            data.robotAxisCovFeedbackKF,
            data.robotAxisValuesReferenceKF,
            data.robotAxisVelocitiesReferenceKF,
            data.robotAxisAccelerationReferenceKF,
            data.robotAxisCovReferenceKF);
    else
        this->hapticModule.m_robotLeftHand->getEstimatedMotorsState(
            data.robotAxisValuesFeedbackKF,
            data.robotAxisVelocitiesFeedbackKF,
            data.robotAxisAccelerationFeedbackKF,
            data.robotAxisCovFeedbackKF,
            data.robotAxisValuesReferenceKF,
            data.robotAxisVelocitiesReferenceKF,
            data.robotAxisAccelerationReferenceKF,
            data.robotAxisCovReferenceKF);

    if (this->isRightHand)
        for (size_t i = 0; i < this->numOfRobotActuatedAxis; i++)
            data.robotAxisError[i] = this->hapticModule.m_icubRightFingerAxisValueError[i];
    else
        for (size_t i = 0; i < this->numOfRobotActuatedAxis; i++)
            data.robotAxisError[i] = this->hapticModule.m_icubLeftFingerAxisValueError[i];

    if (this->isRightHand)
        for (size_t i = 0; i < this->numOfRobotActuatedAxis; i++)
            data.robotAxisErrorSmoothed[i]
                = this->hapticModule.m_icubRightFingerAxisValueErrorSmoothed[i];
    else
        for (size_t i = 0; i < this->numOfRobotActuatedAxis; i++)
            data.robotAxisErrorSmoothed[i]
                = this->hapticModule.m_icubLeftFingerAxisValueErrorSmoothed[i];

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getFingerJointReference(data.robotJointsReference);
    else
        this->hapticModule.m_robotLeftHand->getFingerJointReference(data.robotJointsReference);

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getFingerJointsFeedback(data.robotJointsFeedback);
    else
        this->hapticModule.m_robotLeftHand->getFingerJointsFeedback(data.robotJointsFeedback);

    if (this->isRightHand)
        this->hapticModule.m_robotRightHand->getEstimatedJointState(data.robotJointsFeedbackKF,
                                                                    data.robotJointsExpectedKF);
    else
        this->hapticModule.m_robotLeftHand->getEstimatedJointState(data.robotJointsFeedbackKF,
                                                                   data.robotJointsExpectedKF);

    if (this->isRightHand)
        this->hapticModule.m_gloveRightHand->getHandJointAngles(data.humanJointValues);
    else
        this->hapticModule.m_gloveLeftHand->getHandJointAngles(data.humanJointValues);

    if (this->isRightHand)
        this->hapticModule.m_gloveRightHand->getHandJointAngles(data.humanJointValues);
    else
        this->hapticModule.m_gloveLeftHand->getHandJointAngles(data.humanJointValues);

    data.humanFingertipPose = Eigen::MatrixXd::Zero(this->numOfHumanHandFingers, 7); // to implement
    data.humanForceFeedback.resize(this->numOfHumanHandFingers, 0.0); // to implement
    data.humanVibrotactileFeedback.resize(this->numOfHumanHandFingers, 0.0); // to implement

    if (this->isRightHand)
        this->hapticModule.m_gloveRightHand->getHandPalmRotation(data.humanPalmRotation);
    else
        this->hapticModule.m_gloveLeftHand->getHandPalmRotation(data.humanPalmRotation);

    return true;
}
bool HapticGloveModule::Logger::logData()
{
    if (!this->updateData())
    {
        yError() << "[LoggerImplementation::logData()] cannot update the data";
    }

    this->logger->add("time", this->data.time);

    this->logger->add(this->robotPrefix + "AxisReference", this->data.robotAxisReference);
    this->logger->add(this->robotPrefix + "AxisFeedback", this->data.robotAxisFeedback);

    // to check if it is real robot or simulation
    if (this->hapticModule.m_robot == "icub")
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
    this->logger->add(this->robotPrefix + "AxisError", this->data.robotAxisError);
    this->logger->add(this->robotPrefix + "AxisErrorSmoothed", this->data.robotAxisErrorSmoothed);

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

    return true;
}

bool HapticGloveModule::Logger::closeLogger()
{

#ifdef ENABLE_LOGGER
    this->logger->flush_available_data();
#endif
    yInfo() << "[LoggerImplementation::closeLogger] Logger is closing.";
    return true;
}
