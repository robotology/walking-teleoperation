#include <BilateralTeleoperation.hpp>
#include <Logger.hpp>

// yarp
#include <yarp/os/LogStream.h>

// std
#include <thread>

using namespace HapticGlove;

BilateralTeleoperation::BilateralTeleoperation()
{
    m_logPrefix = "BilateralTeleoperation::";
}

BilateralTeleoperation::~BilateralTeleoperation()
{
}

bool BilateralTeleoperation::configure(const yarp::os::Searchable& config,
                                       const std::string& name,
                                       const bool& rightHand)
{
    m_logPrefix += rightHand ? "RightHand:: " : "LeftHand:: ";

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

    // initialize the robot control class of the left hand
    m_robotLeftHand = std::make_unique<RobotController>();
    if (!m_robotLeftHand->configure(config, name))
    {
        yError() << m_logPrefix << "unable to initialize robot controller.";
        return false;
    }

    // intialize the glove control helper of the left hand
    m_gloveLeftHand = std::make_unique<HapticGlove::GloveControlHelper>();
    if (!m_gloveLeftHand->configure(config, name, rightHand))
    {
        yError() << m_logPrefix << "unable to initialize the glove control helper.";
        return false;
    }

    // initialize the retaregting class
    size_t noRobotAllAxis = m_robotLeftHand->controlHelper()->getNumberOfAllAxis();
    size_t noRobotActuatedAxis = m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis();
    std::vector<std::string> robotActuatedJointNameList;
    m_robotLeftHand->controlHelper()->getActuatedJointNameList(robotActuatedJointNameList);
    std::vector<std::string> robotActuatedAxisNameList;
    m_robotLeftHand->controlHelper()->getActuatedAxisNameList(robotActuatedAxisNameList);

    size_t noBuzzMotors = m_gloveLeftHand->getNumOfVibrotactileFeedbacks();
    std::vector<std::string> humanJointNameList;
    m_gloveLeftHand->getHumanHandJointsNames(humanJointNameList);

    m_retargetingLeftHand = std::make_unique<HapticGlove::Retargeting>(
        robotActuatedJointNameList, robotActuatedAxisNameList, humanJointNameList);
    if (!m_retargetingLeftHand->configure(config, name, rightHand))
    {
        yError() << m_logPrefix << "m_retargetingLeftHand->configure returns false";
        return false;
    }

    // initialize the vectors
    // vectors associated with robot
    m_icubLeftFingerJointsReference.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints(), 0.0);
    m_icubLeftFingerJointsFeedback.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints(), 0.0);

    m_icubLeftFingerAxisValueReference.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
    m_icubLeftFingerAxisValueFeedback.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);

    m_icubLeftFingerAxisVelocityReference.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
    m_icubLeftFingerAxisVelocityFeedback.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);

    m_icubLeftFingerAxisValueError.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
    m_icubLeftFingerAxisVelocityError.resize(
        m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);

    // vectors associated with human
    m_gloveLeftBuzzMotorReference.resize(m_gloveLeftHand->getNumOfVibrotactileFeedbacks(), 0.0);
    m_gloveLeftForceFeedbackReference.resize(m_gloveLeftHand->getNumOfForceFeedback(), 0.0);

    // initialize the member variables
    m_timePreparation = 0.0;
    m_timeNow = 0.0;
    m_timeConfigurationEnding = yarp::os::Time::now();

    m_Value_error_threshold_transient = 0.1;

    // logger
    m_enableLogger = config.check("enableLogger", yarp::os::Value(0)).asBool();

    // print information
    yInfo() << m_logPrefix << "enable the logger: " << m_enableLogger;

    //
    if (!m_gloveLeftHand->setupGlove())
    {
        yError() << m_logPrefix << "cannot setup the glove.";
        return false;
    }

    //
    yInfo() << m_logPrefix << "configuration is done. ";

    return true;
}

bool BilateralTeleoperation::getFeedbacks()
{
    // get feedback from the robot left hand values
    if (!m_robotLeftHand->updateFeedback())
    {
        yError() << m_logPrefix << "unable to update the feedback values of the left hand fingers.";
    }

    m_robotLeftHand->getFingerAxisFeedback(m_icubLeftFingerAxisValueFeedback);

    m_robotLeftHand->getFingerAxisValueReference(m_icubLeftFingerAxisValueReference);

    m_robotLeftHand->getFingerAxisVelocityFeedback(m_icubLeftFingerAxisVelocityFeedback);

    m_robotLeftHand->getFingerJointsFeedback(m_icubLeftFingerJointsFeedback);

    m_robotLeftHand->estimateNextStates();

    if (m_robotLeftHand->areEstimatorsInitialized())
    {
        m_robotLeftHand->getEstimatedMotorsState(m_axisFeedbackValuesEstimationKF,
                                                 m_axisFeedbackVelocitiesEstimationKF,
                                                 m_axisFeedbackAccelerationEstimationKF,
                                                 m_feedbackAxisCovEstimationKF,
                                                 m_axisReferenceValuesEstimationKF,
                                                 m_axisReferenceVelocitiesEstimationKF,
                                                 m_axisReferenceAccelerationEstimationKF,
                                                 m_referenceAxisCovEstimationKF);
    }

    return true;
}

bool BilateralTeleoperation::run()
{
    // initialize the estimator in case it is not initialized
    if (!m_robotLeftHand->areEstimatorsInitialized())
    {
        if (!m_robotLeftHand->initializeEstimators())
        {
            yError() << m_logPrefix << "cannot initialize the left robot hand.";
            return false;
        }
    }

    // feedbacks and estimators
    if (!this->getFeedbacks())
    {
        yError() << m_logPrefix << "unable to get the feedback";
        return false;
    }

    // retarget human motion to the robot
    std::vector<double> humanJointAngles;
    m_gloveLeftHand->getHandJointAngles(humanJointAngles);
    if (!m_retargetingLeftHand->retargetHumanMotionToRobot(humanJointAngles))
    {
        yError() << m_logPrefix << "unable to retaget human motion to robot motions.";
        return false;
    }
    m_retargetingLeftHand->getRobotJointReferences(m_icubLeftFingerJointsReference);

    // compute the force feedback
    m_retargetingLeftHand->retargetHapticFeedbackFromRobotToHuman(
        m_axisReferenceValuesEstimationKF,
        m_axisReferenceVelocitiesEstimationKF,
        m_axisFeedbackValuesEstimationKF,
        m_axisFeedbackVelocitiesEstimationKF);

    m_retargetingLeftHand->getForceFeedbackToHuman(m_gloveLeftForceFeedbackReference);
    m_retargetingLeftHand->getVibrotactileFeedbackToHuman(m_gloveLeftBuzzMotorReference);

    m_robotLeftHand->setFingersJointReference(m_icubLeftFingerJointsReference);
    if (m_moveRobot)
    {
        m_robotLeftHand->move();
        m_gloveLeftHand->setFingertipForceFeedbackReferences(m_gloveLeftForceFeedbackReference);
        m_gloveLeftHand->setFingertipVibrotactileFeedbackReferences(m_gloveLeftBuzzMotorReference);
    }

    if (m_enableLogger)
    {
        // TO COMPLETE
    }

    return true;
}
bool BilateralTeleoperation::prepare(bool& isPrepared)
{

    isPrepared = false;
    if (!this->getFeedbacks())
    {
        yError() << m_logPrefix << "unable to get the feedback";
        return false;
    }

    m_timePreparation = yarp::os::Time::now();
    yInfo() << "time collecting data:" << m_timePreparation - m_timeConfigurationEnding;
    int dTime = int((m_timePreparation - m_timeConfigurationEnding) / m_dT);
    int CouplingConstant = (int)(m_calibrationTimePeriod / m_dT);

    int axisNumber = int(dTime / CouplingConstant); // both operands needs to be integer
                                                    //        axisNumber = dTime / 500;

    if (axisNumber >= m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis())
    {
        yInfo() << "******>>>> Data collected ...";
        if (!m_robotLeftHand->isRobotPrepared())
        {
            if (!m_robotLeftHand->trainCouplingMatrix())
            {
                yInfo() << "cannot claibrate the coupling matrix and find the "
                           "coefficient matrix";
                return false;
            }
        }
        if (m_getHumanMotionRange)
        {
            std::vector<double> humanHandJointRangeMin, humanHandJointRangeMax;
            m_gloveLeftHand->getHumanFingerJointsMotionRange(humanHandJointRangeMin,
                                                             humanHandJointRangeMax);
            m_retargetingLeftHand->computeJointAngleRetargetingParams(humanHandJointRangeMin,
                                                                      humanHandJointRangeMax);
        }

        //                m_timePreparationStarting = yarp::os::Time::now();
        //                m_state = HapticGloveFSM::Running;

    } else
    {
        double time = double(dTime % CouplingConstant) * m_dT * (M_PI / m_calibrationTimePeriod);
        //                time = double(dTime % 500) / 500.0 * (M_PI * 2.0);

        m_robotLeftHand->LogDataToCalibrateRobotMotorsJointsCouplingSin(time, axisNumber);
        if (m_getHumanMotionRange)
        {
            m_gloveLeftHand->findHumanMotionRange();
        }
    }

    if (m_robotLeftHand->isRobotPrepared())
    {
        isPrepared = true;
    }

    return true;
}

bool BilateralTeleoperation::close()
{
    m_gloveLeftHand->stopHapticFeedback();
    m_gloveLeftHand->stopVibrotactileFeedback();
    m_gloveLeftHand->stopForceFeedback();

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 100ms.

    return true;
}
