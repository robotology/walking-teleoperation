/**
 * @file Teleoperation.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <Logger.hpp>
#include <Teleoperation.hpp>

#define _USE_MATH_DEFINES // for C++
// std
#include <cmath>
// yarp
#include <yarp/os/LogStream.h>

// std
#include <thread>

using namespace HapticGlove;

Teleoperation::Teleoperation()
{
    m_logPrefix = "Teleoperation::";
}

Teleoperation::~Teleoperation()
{
}

bool Teleoperation::configure(const yarp::os::Searchable& config,
                              const std::string& name,
                              const bool& rightHand)
{
    m_logPrefix += rightHand ? "RightHand:: " : "LeftHand:: ";

    m_robot = name;

    // get the period
    m_dT = config.check("samplingTime", yarp::os::Value(0.1)).asDouble();

    // check the calibration time period
    m_calibrationTimePeriod
        = config.check("calibrationTimePeriod", yarp::os::Value(10.0)).asDouble();
    yInfo() << m_logPrefix << "calibration time period: " << m_calibrationTimePeriod;

    m_moveRobot = config.check("enableMoveRobot", yarp::os::Value(1)).asBool();
    yInfo() << m_logPrefix << "move the robot: " << m_moveRobot;

    // check if perform calibration phase for geting the user motion range
    m_getHumanMotionRange = config.check("getHumanMotionRange", yarp::os::Value(0)).asBool();

    // initialize the robot controller object
    m_robotController = std::make_unique<RobotController>();
    if (!m_robotController->configure(config, m_robot, rightHand))
    {
        yError() << m_logPrefix << "unable to initialize robot controller.";
        return false;
    }

    // intialize the human glove object
    m_humanGlove = std::make_unique<HapticGlove::GloveControlHelper>();
    if (!m_humanGlove->configure(config, m_robot, rightHand))
    {
        yError() << m_logPrefix << "unable to initialize the glove control helper.";
        return false;
    }

    // initialize the retaregting object
    std::vector<std::string> robotActuatedJointNameList;
    std::vector<std::string> robotActuatedAxisNameList;
    std::vector<std::string> humanJointNameList;

    m_robotController->controlHelper()->getActuatedJointNames(robotActuatedJointNameList);
    m_robotController->controlHelper()->getActuatedAxisNames(robotActuatedAxisNameList);
    m_humanGlove->getHumanHandJointsNames(humanJointNameList);

    m_retargeting = std::make_unique<HapticGlove::Retargeting>(
        robotActuatedJointNameList, robotActuatedAxisNameList, humanJointNameList);
    if (!m_retargeting->configure(config, m_robot, rightHand))
    {
        yError() << m_logPrefix << "unable to initialize retargeting class.";
        return false;
    }

    std::vector<double> minLimits, maxLimits;
    if (!m_robotController->controlHelper()->getLimits(minLimits, maxLimits))
    {
        yError() << m_logPrefix << "unable to get the limits from the robot controller.";
        return false;
    }

    if (!m_retargeting->setRobotAxisLimits(minLimits, maxLimits))
    {
        yError() << m_logPrefix
                 << "unable to set limits ofrobot actuated axes for retargeting class.";
        return false;
    }

    // get the vector sizes
    const size_t numRobotAllAxis = m_robotController->controlHelper()->getNumberOfAllAxis();
    const size_t numRobotActuatedAxis
        = m_robotController->controlHelper()->getNumberOfActuatedAxis();
    const size_t numRobotActuatedJoints
        = m_robotController->controlHelper()->getNumberOfActuatedJoints();
    const size_t numHumanHandJoints = m_humanGlove->getNumOfHandJoints();
    const size_t numHumanForceFeedback = m_humanGlove->getNumOfHandJoints();
    const size_t numHumanVibrotactileFeedback = m_humanGlove->getNumOfHandJoints();

    // initialize the vectors
    // robot
    m_data.robotAxisReferences.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisFeedbacks.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisVelocityFeedbacks.resize(numRobotActuatedAxis, 0.0);

    m_data.robotJointReferences.resize(numRobotActuatedJoints, 0.0);
    m_data.robotJointFeedbacks.resize(numRobotActuatedJoints, 0.0);

    m_data.robotAxisValueErrors.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisVelocityErrors.resize(numRobotActuatedAxis, 0.0);

    m_data.robotAxisValueFeedbacksKf.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisVelocityFeedbacksKf.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisAccelerationFeedbacksKf.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisCovFeedbacksKf = Eigen::MatrixXd::Zero(numRobotActuatedAxis, 9);

    m_data.robotAxisValueReferencesKf.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisVelocityReferencesKf.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisAccelerationReferencesKf.resize(numRobotActuatedAxis, 0.0);
    m_data.robotAxisCovReferencesKf = Eigen::MatrixXd::Zero(numRobotActuatedAxis, 9);

    // human
    m_data.humanJointValues.resize(numHumanHandJoints, 0.0);
    m_data.humanVibrotactileFeedbacks.resize(numHumanVibrotactileFeedback, 0.0);
    m_data.humanForceFeedbacks.resize(numHumanForceFeedback, 0.0);

    // set up the glove
    if (!m_humanGlove->setupGlove())
    {
        yError() << m_logPrefix << "cannot setup the glove.";
        return false;
    }

    // logger
    m_enableLogger = config.check("enableLogger", yarp::os::Value(0)).asBool();
    if (m_enableLogger)
    {
        m_loggerLeftHand = std::make_unique<Logger>(*this, rightHand);
        if (!m_loggerLeftHand->openLogger())
        {
            yError() << m_logPrefix << "unable to open the logger.";
            return false;
        }
    }

    // print information:
    yInfo() << m_logPrefix << "enable the logger: " << m_enableLogger;
    yInfo() << m_logPrefix << "configuration is done. ";

    // update the end of the configuration time step
    m_timeConfigurationEnd = yarp::os::Time::now();

    return true;
}

bool Teleoperation::getFeedbacks()
{
    // get feedback from the robot left hand values
    if (!m_robotController->updateFeedback())
    {
        yWarning() << m_logPrefix << "unable to update the feedback values of the robot.";
    }

    if (!m_robotController->estimateNextStates())
    {
        yWarning() << m_logPrefix << "unable to perform the estimation.";
    }

    m_robotController->getAxisValueReferences(m_data.robotAxisReferences);

    m_robotController->getAxisValueFeedbacks(m_data.robotAxisFeedbacks);

    m_robotController->getAxisVelocityFeedbacks(m_data.robotAxisVelocityFeedbacks);

    m_robotController->getJointValueFeedbacks(m_data.robotJointFeedbacks);

    // get the estimation values
    m_robotController->getEstimatedMotorsState(m_data.robotAxisValueFeedbacksKf,
                                               m_data.robotAxisVelocityFeedbacksKf,
                                               m_data.robotAxisAccelerationFeedbacksKf,
                                               m_data.robotAxisCovFeedbacksKf,
                                               m_data.robotAxisValueReferencesKf,
                                               m_data.robotAxisVelocityReferencesKf,
                                               m_data.robotAxisAccelerationReferencesKf,
                                               m_data.robotAxisCovReferencesKf);

    return true;
}

bool Teleoperation::run()
{

    // retarget human motion to the robot
    if (!m_humanGlove->getHandJointAngles(m_data.humanJointValues))
    {
        yWarning() << m_logPrefix << "unable to get human latest joint angles.";
    }

    if (!m_retargeting->retargetHumanMotionToRobot(m_data.humanJointValues))
    {
        yWarning() << m_logPrefix << "unable to retaget human motion to robot motions.";
    }

    if (!m_retargeting->getRobotJointReferences(m_data.robotJointReferences))
    {
        yWarning() << m_logPrefix << "unable to get the robot joint references from retargeting.";
    }

    if (!m_robotController->setJointReferences(m_data.robotJointReferences))
    {
        yWarning() << m_logPrefix << "unable to set the joint references to the robot.";
    }

    // since we have estimators for the references, we put the getFeedback method at this point.
    if (!this->getFeedbacks())
    {
        yWarning() << m_logPrefix << "unable to get the feedback";
    }

    if (!m_robotController->computeControlSignals())
    {
        yWarning() << m_logPrefix << "unable to compute the control signals.";
    }

    // compute the haptic feedback
    if (!m_retargeting->retargetHapticFeedbackFromRobotToHuman(m_data.robotAxisValueReferencesKf,
                                                               m_data.robotAxisVelocityReferencesKf,
                                                               m_data.robotAxisValueFeedbacksKf,
                                                               m_data.robotAxisVelocityFeedbacksKf))
    {
        yWarning() << m_logPrefix
                   << "unable to retarget haptic feedback from the robot to the human.";
    }

    if (!m_retargeting->getForceFeedbackToHuman(m_data.humanForceFeedbacks))
    {
        yWarning() << m_logPrefix << "unable to get the force feedback from retargeting.";
    }

    if (!m_retargeting->getVibrotactileFeedbackToHuman(m_data.humanVibrotactileFeedbacks))
    {
        yWarning() << m_logPrefix << "unable to get the vibrotactile feedback from retargeting.";
    }

    // set the values
    if (m_moveRobot)
    {
        m_robotController->move();
        m_humanGlove->setFingertipForceFeedbackReferences(m_data.humanForceFeedbacks);
        m_humanGlove->setFingertipVibrotactileFeedbackReferences(m_data.humanVibrotactileFeedbacks);
    }

    if (m_enableLogger)
    {
        if (!m_loggerLeftHand->logData())
        {
            yWarning() << m_logPrefix << "unable to log the data.";
        }
    }

    return true;
}
bool Teleoperation::prepare(bool& isPrepared)
{

    isPrepared = false;
    if (!this->getFeedbacks())
    {
        yError() << m_logPrefix << "unable to get the feedback";
        return false;
    }

    double time = yarp::os::Time::now();
    yInfo() << "time collecting data: " << time - m_timeConfigurationEnd;
    int dTime = int((time - m_timeConfigurationEnd) / m_dT);
    int CouplingConstant = (int)(m_calibrationTimePeriod / m_dT);

    int axisNumber = int(dTime / CouplingConstant); // both operands needs to be integer
                                                    //        axisNumber = dTime / 500;

    if (axisNumber >= m_robotController->controlHelper()->getNumberOfActuatedAxis())
    {
        yInfo() << m_logPrefix << "data collected to learn robot model.";
        if (!m_robotController->isRobotPrepared())
        {
            if (!m_robotController->trainCouplingMatrix())
            {
                yInfo() << "cannot train the coupling matrix and find the "
                           "coefficient matrix";
                return false;
            }
        }
        if (m_getHumanMotionRange)
        {
            std::vector<double> humanHandJointRangeMin, humanHandJointRangeMax;
            m_humanGlove->getHumanFingerJointsMotionRange(humanHandJointRangeMin,
                                                          humanHandJointRangeMax);
            m_retargeting->computeJointAngleRetargetingParams(humanHandJointRangeMin,
                                                              humanHandJointRangeMax);
        }
    } else
    {
        double time = double(dTime % CouplingConstant) * m_dT * (M_PI / m_calibrationTimePeriod);
        //                time = double(dTime % 500) / 500.0 * (M_PI * 2.0);

        m_robotController->LogDataToCalibrateRobotAxesJointsCoupling(time, axisNumber);
        if (m_getHumanMotionRange)
        {
            m_humanGlove->findHumanMotionRange();
        }
    }

    if (m_robotController->isRobotPrepared())
    {
        isPrepared = true;
    }

    // initialize the estimator in case it is not initialized
    if (m_robotController->areEstimatorsInitialized())
    {
        yError() << m_logPrefix << "at this point estimators are not supposed to be initialized.";
        return false;
    }
    if (isPrepared)
    {
        if (!m_robotController->initializeEstimators())
        {
            yError() << m_logPrefix << "cannot initialize the robot estimators.";
            return false;
        }
    }
    return true;
}

bool Teleoperation::close()
{
    // close the logger.
    yInfo() << m_logPrefix << "trying to close.";

    if (m_enableLogger)
    {
        if (!m_loggerLeftHand->closeLogger())
        {
            yError() << m_logPrefix << "unable to close the logger.";
            return false;
        }
    }
    m_humanGlove->stopHapticFeedback();

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 100ms.

    if (!m_robotController->controlHelper()->close())
    {
        yError() << m_logPrefix << "unable to close the robot controller.";
        return false;
    }

    if (!m_humanGlove->close())
    {
        yError() << m_logPrefix << "unable to close the human  and glove control helper.";
        return false;
    }

    if (!m_retargeting->close())
    {
        yError() << m_logPrefix << "unable to close the retargeting.";
        return false;
    }

    yInfo() << m_logPrefix << "closed successfully.";

    return true;
}
