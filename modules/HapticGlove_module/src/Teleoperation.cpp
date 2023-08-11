// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <Logger.hpp>
#include <Teleoperation.hpp>

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

Teleoperation::~Teleoperation() = default;

bool Teleoperation::configure(const yarp::os::Searchable& config,
                              const std::string& name,
                              const bool& rightHand)
{
    m_logPrefix += rightHand ? "RightHand:: " : "LeftHand:: ";

    std::string portPrefix = "/";
    portPrefix += rightHand ? "RightHand" : "LeftHand";

    m_robot = name;

    // get the period
    m_dT = config.check("samplingTime", yarp::os::Value(0.1)).asFloat64();

    // check the calibration time period
    m_calibrationTimePeriod
        = config.check("calibrationTimePeriod", yarp::os::Value(10.0)).asFloat64();
    yInfo() << m_logPrefix << "calibration time period: " << m_calibrationTimePeriod;

    m_moveRobot = config.check("enableMoveRobot", yarp::os::Value(1)).asBool();
    yInfo() << m_logPrefix << "move the robot: " << m_moveRobot;

    m_useSkin = config.check("useSkin", yarp::os::Value(1)).asBool();
    yInfo() << m_logPrefix << "use the robot fingertip skin: " << m_useSkin;

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

    std::vector<double> minAxisLimits, maxAxisLimits;
    if (!m_robotController->controlHelper()->getActuatedAxisLimits(minAxisLimits, maxAxisLimits))
    {
        yError() << m_logPrefix << "unable to get the axis limits from the robot controller.";
        return false;
    }

    if (!m_retargeting->setRobotAxisLimits(minAxisLimits, maxAxisLimits))
    {
        yError() << m_logPrefix
                 << "unable to set limits of robot actuated axes for retargeting class.";
        return false;
    }

    std::vector<double> minJointLimits, maxJointLimits;
    if (!m_robotController->controlHelper()->getActuatedJointLimits(minJointLimits, maxJointLimits))
    {
        yError() << m_logPrefix << "unable to get the joint limits from the robot controller.";
        return false;
    }

    if (!m_retargeting->setRobotJointLimits(minJointLimits, maxJointLimits))
    {
        yError() << m_logPrefix
                 << "unable to set limits of robot actuated joints for retargeting class.";
        return false;
    }

    if (m_useSkin)
    {
        m_robotSkin = std::make_unique<HapticGlove::RobotSkin>();
        if (!m_robotSkin->configure(config, m_robot, rightHand))
        {
            yError() << m_logPrefix << "unable to configure robot skin class.";
            return false;
        }
    }

    // get the vector sizes
    const size_t numRobotAllAxis = m_robotController->controlHelper()->getNumberOfAllAxis();
    const size_t numRobotActuatedAxis
        = m_robotController->controlHelper()->getNumberOfActuatedAxis();
    const size_t numRobotActuatedJoints
        = m_robotController->controlHelper()->getNumberOfActuatedJoints();
    const size_t numRobotFingers = m_robotController->controlHelper()->getNumberOfRobotFingers();

    const size_t numHumanHandJoints = m_humanGlove->getNumOfHandJoints();
    const size_t numHumanForceFeedback = m_humanGlove->getNumOfForceFeedback();
    const size_t numHumanVibrotactileFeedback = m_humanGlove->getNumOfVibrotactileFeedbacks();

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

    // skin
    m_data.doRobotFingerSkinsWork.resize(numRobotFingers, 0.0);
    m_data.robotFingerSkinAbsoluteValueVibrotactileFeedbacks.resize(numRobotFingers, 0.0);
    m_data.areFingersSkinInContact.resize(numRobotFingers, 0.0);
    m_data.robotFingerSkinDerivativeValueVibrotactileFeedbacks.resize(numRobotFingers, 0.0);
    m_data.robotFingerSkinTotalValueVibrotactileFeedbacks.resize(numRobotFingers, 0.0);

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

    // Initialize RPC
    std::string rpcPortName = "/hapticGloveTeleoperation/rpc";
    if(!m_rpcPort.open(portPrefix + rpcPortName))
    {
        yWarning() << m_logPrefix << "Failed to open" << portPrefix + rpcPortName;
    }
    else
    {
        if (!this->yarp().attachAsServer(m_rpcPort)) {
            yWarning() << m_logPrefix << "Failed to attach" << portPrefix + rpcPortName << "to the RPC service";
        }
    }

    // print information:
    yInfo() << m_logPrefix << "enable the logger: " << m_enableLogger;
    yInfo() << m_logPrefix << "configuration is done. ";

    return true;
}

bool Teleoperation::enableMoveRobot(const bool value)
{

    std::lock_guard<std::mutex> lock(m_mutex);

    m_moveRobot = value;

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

    // get tactile sensors data
    if (m_useSkin)
    {
        m_robotSkin->updateTactileFeedbacks();
    }
    return true;
}

bool Teleoperation::run()
{

    std::lock_guard<std::mutex> lock(m_mutex);

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
    if (!m_retargeting->retargetHapticFeedbackFromRobotToHumanUsingKinestheticData(
            m_data.robotAxisValueReferencesKf,
            m_data.robotAxisVelocityReferencesKf,
            m_data.robotAxisValueFeedbacksKf,
            m_data.robotAxisVelocityFeedbacksKf))
    {
        yWarning() << m_logPrefix
                   << "unable to retarget haptic feedback from the robot to the human.";
    }

    if (m_useSkin)
    {
        // since skins may stop working in the middle of an experiment, we check it continuously
        m_robotSkin->doTactileSensorsWork(m_data.doRobotFingerSkinsWork);

        // check if the fingers are in contact
        m_robotSkin->areFingersInContact(m_data.areFingersSkinInContact);

        // get the skin data
        // to delete
        m_robotSkin->getVibrotactileAbsoluteFeedback(
            m_data.robotFingerSkinAbsoluteValueVibrotactileFeedbacks);

        //        yInfo() << "tactile absolute: " <<
        //        m_data.robotFingerSkinAbsoluteValueVibrotactileFeedbacks;

        // to delete
        m_robotSkin->getVibrotactileDerivativeFeedback(
            m_data.robotFingerSkinDerivativeValueVibrotactileFeedbacks);
        //        yInfo() << "tactile derivative: "
        //                << m_data.robotFingerSkinDerivativeValueVibrotactileFeedbacks;

        m_robotSkin->getVibrotactileTotalFeedback(
            m_data.robotFingerSkinTotalValueVibrotactileFeedbacks);
        //        yInfo() << "tactile total: " <<
        //        m_data.robotFingerSkinTotalValueVibrotactileFeedbacks;

        // compute haptic feedback with consideration of the skin
        m_retargeting->retargetHapticFeedbackFromRobotToHumanUsingSkinData(
            m_data.doRobotFingerSkinsWork,
            m_data.areFingersSkinInContact,
            m_data.robotFingerSkinTotalValueVibrotactileFeedbacks);
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
    int dTime = int((time - m_timeConfigurationEnd) / m_dT);
    int CouplingConstant = (int)(m_calibrationTimePeriod / m_dT);

    int axisNumber = int(dTime / CouplingConstant); // both operands needs to be integer
//    yInfo() << m_logPrefix << "time [steps]: " << dTime << ", axis number: " << axisNumber;

    if (axisNumber >= m_robotController->controlHelper()->getNumberOfActuatedAxis())
    {
        yInfo() << m_logPrefix << "data collected to learn robot model.";
        // robot
        if (!m_robotController->isRobotPrepared())
        {
            if (!m_robotController->trainCouplingMatrix())
            {
                yInfo() << m_logPrefix
                        << "cannot train the coupling matrix and find the coefficient matrix";
                return false;
            }
        }
        // human
        if (m_getHumanMotionRange)
        {
            std::vector<double> humanHandJointRangeMin, humanHandJointRangeMax;
            m_humanGlove->getHumanFingerJointsMotionRange(humanHandJointRangeMin,
                                                          humanHandJointRangeMax);
            m_retargeting->computeJointAngleRetargetingParams(humanHandJointRangeMin,
                                                              humanHandJointRangeMax);
        }
        // skin
        if (m_useSkin)
        {
            m_robotSkin->computeCalibrationParamters();
        }

        if (m_robotController->isRobotPrepared())
        {
           isPrepared = true;
        }

    } else
    {

        double time = double(dTime % CouplingConstant) * m_dT * (M_PI / m_calibrationTimePeriod);
        // robot
        m_robotController->LogDataToCalibrateRobotAxesJointsCoupling(time, axisNumber);
        // human

        if (m_getHumanMotionRange)
        {
            m_humanGlove->findHumanMotionRange();
        }

        // skin
        if (m_useSkin)
        {
            m_robotSkin->collectSkinDataForCalibration();
        }
    }

    // initialize the estimator in case it is not initialized
    if (m_robotController->areEstimatorsInitialized())
    {
        yWarning() << m_logPrefix << "at this point estimators are not supposed to be initialized.";
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

bool Teleoperation::wait()
{
    bool ok = true;

    if (!m_humanGlove->stopHapticFeedback())
    {
        yWarning() << m_logPrefix << "cannot stop haptic feedback.";
        ok &= false;
    }

    return ok;
}

bool Teleoperation::close()
{
    // close the logger.
    yInfo() << m_logPrefix << "trying to close.";
    bool ok = true;

    if (m_enableLogger)
    {
        if (!m_loggerLeftHand->closeLogger())
        {
            yWarning() << m_logPrefix << "unable to close the logger.";
            ok &= false;
        }
    }
    if (!m_humanGlove->stopHapticFeedback())
    {
        yWarning() << m_logPrefix << "cannot stop haptic feedback.";
        ok &= false;
    }

    // close RPC port
    m_rpcPort.close();

    //  in order to be sure the stop haptic command is sent before closing the module, otherwise the
    //  glove may continue to provide the haptic feedback according to the last sent command.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(50)); // wait for 50 ms to send the stop command.
    if (m_useSkin)
    {
        if (!m_robotSkin->close())
        {
            yWarning() << m_logPrefix << "unable to close the skin class.";
            ok &= false;
        }
    }

    if (!m_robotController->controlHelper()->close())
    {
        yWarning() << m_logPrefix << "unable to close the robot controller.";
        ok &= false;
    }

    if (!m_humanGlove->close())
    {
        yWarning() << m_logPrefix << "unable to close the human and glove control helper.";
        ok &= false;
    }

    if (!m_retargeting->close())
    {
        yWarning() << m_logPrefix << "unable to close the retargeting.";
        ok &= false;
    }

    yInfo() << m_logPrefix << "closed" << (ok ? "Successfully" : "badly") << ".";

    return ok;
}

void Teleoperation::setEndOfConfigurationTime(const double& time)
{
    m_timeConfigurationEnd = time;
}
