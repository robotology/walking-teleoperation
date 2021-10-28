/**
 * @file RobotController_hapticGlove.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// teleoperation
#include <RobotController_hapticGlove.hpp>
#include <Utils.hpp>

// std
#include <thread>

using namespace HapticGlove;

bool RobotController::configure(const yarp::os::Searchable& config,
                                const std::string& name,
                                const bool& rightHand)
{
    m_rightHand = rightHand;

    m_logPrefix = "RobotController::";
    m_logPrefix += m_rightHand ? "RightHand:: " : "LeftHand:: ";

    m_robotControlInterface = std::make_unique<RobotControlInterface>();
    if (!m_robotControlInterface->configure(config, name, false))
    {
        yError() << m_logPrefix << "unable to intialized and configure the control.";
        return false;
    }

    m_numAllAxis = m_robotControlInterface->getNumberOfAllAxis();
    m_numActuatedAxis = m_robotControlInterface->getNumberOfActuatedAxis();

    m_numAllJoints = m_robotControlInterface->getNumberOfAllJoints();
    m_numActuatedJoints = m_robotControlInterface->getNumberOfActuatedJoints();

    // check if the axes and joints are coupled
    m_axesJointsCoupled = config.check("motorsJointsCoupled", yarp::os::Value(0)).asBool();

    if (!m_axesJointsCoupled)
    {
        if (m_numActuatedAxis != m_numActuatedJoints)
        {
            // in the case we assume it is NOT underactuated.
            yError() << m_logPrefix
                     << "the robot joints and axes are not coupled, so the number actuated axes "
                        "and actuated joints should be equal; m_numActuatedAxis: "
                     << m_numActuatedAxis << "m_numActuatedJoints: " << m_numActuatedJoints;
            return false;
        }
    }

    // check if we need to do calibration
    m_doCalibration = config.check("doCalibration", yarp::os::Value(0)).asBool();

    m_kGain = config.check("exponentialFilterGain", yarp::os::Value(0.9)).asDouble();

    if (m_axesJointsCoupled)
    {
        m_A.resize(m_numActuatedJoints, m_numActuatedAxis);
        m_Bias.resize(m_numActuatedJoints, 1);

        if (!m_doCalibration)
        {
            if (m_numActuatedAxis != m_numAllAxis && m_numActuatedJoints != m_numAllJoints)
            {
                yError() << m_logPrefix
                         << "if you do not want to learn the model of coupling betwen robot joints "
                            "and axes, all of them should be actuated. You have two options: \n"
                            "    - learn the model: enable the option `doCalibration` \n"
                            "    - use all the axes for the robot control";
                return false;
            }
            std::vector<double> A_vector, Bias_vector;
            A_vector.resize(m_numActuatedJoints * m_numActuatedAxis);
            Bias_vector.resize(m_numActuatedJoints);
            if (!YarpHelper::getVectorFromSearchable(config, "CouplingMatrix", A_vector))
            {
                yError() << m_logPrefix
                         << "initialization failed while reading CouplingMatrix vector.";
                return false;
            }
            if (A_vector.size() != m_numActuatedJoints * m_numActuatedAxis)
            {
                yError() << m_logPrefix
                         << "the size of the A_vector (CouplingMatrix) and (m_numActuatedJoints * "
                            "m_numActuatedAxis) are not equal.";
                return false;
            }
            if (!YarpHelper::getVectorFromSearchable(config, "CouplingBias", Bias_vector))
            {
                yError() << "initialization failed while reading "
                            "CouplingBias vector.";
                return false;
            }
            if (Bias_vector.size() != m_numActuatedJoints)
            {
                yError() << m_logPrefix
                         << "the size of the Bias_vector (CouplingBias) and m_numActuatedJoints "
                            "are not equal.";
                return false;
            }

            size_t idx = 0;
            for (size_t i = 0; i < m_numActuatedJoints; i++)
            {
                for (size_t j = 0; j < m_numActuatedAxis; j++)
                {
                    m_A(i, j) = A_vector[idx];
                    idx++;
                }
                m_Bias(i) = Bias_vector[i];
            }
        }
    } else
    {
        // if not coupled., the mapping between the motors and joints are identity matrix
        m_A = Eigen::MatrixXd::Identity(m_numActuatedAxis, m_numActuatedAxis);
        m_Bias = Eigen::MatrixXd::Zero(m_numActuatedJoints, 1);
    }

    /*
     * commented to simplify the process of having custom set of motors to control.
    yarp::sig::Vector Q_vector(noAllJoints, 0.0), R_vector(noActuatedAxis, 0.0);

    if (!YarpHelper::getYarpVectorFromSearchable(config, "q_matrix_joint_motor", Q_vector))
    {
        yError() << "[RobotController::configure] Initialization failed while reading "
                    "q_matrix_joint_motor vector.";
        return false;
    }

    if (!YarpHelper::getYarpVectorFromSearchable(config, "r_matrix_joint_motor", R_vector))
    {
        yError() << "[RobotController::configure] Initialization failed while reading "
                    "r_matrix_joint_motor vector.";
        return false;
    }

    m_Q = Eigen::MatrixXd::Identity(noActuatedJoints, noActuatedJoints);
    m_R = Eigen::MatrixXd::Zero(noActuatedAxis, noActuatedAxis);
    for (int i = 0; i < noActuatedJoints; i++)
    {
        m_Q(i, i) = Q_vector(i);
    }
    for (int i = 0; i < noActuatedAxis; i++)
    {
        m_R(i, i) = R_vector(i);
    }
*/
    // get control  gains from configuration files
    std::vector<std::string> allAxisNames, actuatedAxisNames;
    std::vector<std::string> allJointNames, actuatedJointNames;
    m_robotControlInterface->getAllAxisNames(allAxisNames);
    m_robotControlInterface->getAllAxisNames(actuatedAxisNames);
    m_robotControlInterface->getAllJointNames(allJointNames);
    m_robotControlInterface->getActuatedJointNames(actuatedJointNames);

    std::vector<double> q, r;
    std::vector<double> qTmp, rTmp;

    if (!YarpHelper::getVectorFromSearchable(config, "q_qp_control", q))
    {
        yError() << m_logPrefix << "unable to find the q_qp_control";
        return false;
    }
    if (!YarpHelper::getVectorFromSearchable(config, "r_qp_control", r))
    {
        yError() << m_logPrefix << "unable to find the r_qp_control";
        return false;
    }
    this->getCustomSetIndices(allJointNames, actuatedJointNames, q, qTmp);
    this->getCustomSetIndices(allAxisNames, actuatedAxisNames, r, rTmp);

    m_Q = Eigen::MatrixXd::Identity(m_numActuatedJoints, m_numActuatedJoints);
    m_R = Eigen::MatrixXd::Identity(m_numActuatedAxis, m_numActuatedAxis);

    for (size_t i = 0; i < m_numActuatedJoints; i++)
        m_Q(i, i) = qTmp[i];
    for (size_t i = 0; i < m_numActuatedAxis; i++)
        m_R(i, i) = rTmp[i];

    // check if robot is working
    bool tmp = false;
    for (size_t i = 0; i < 10; i++)
    {
        if (!updateFeedback())
        {
            yWarning() << m_logPrefix << "unable to get feedback: " << i
                       << "'th trial, waiting for 10 ms.";
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // wait for 100ms.

        } else
        {
            tmp = true;
            break;
        }
    }
    if (!tmp)
    {
        yError() << m_logPrefix << "unable to get feedback from the robot";
        return false;
    }

    //    yarp::sig::Vector buff(m_numActuatedAxis, 0.0);
    //    getFingerAxisFeedback(buff);
    //    yarp::sig::Matrix limits(m_numActuatedAxis, 2);
    //    if (!m_robotControlInterface->getLimits(limits))
    //    {
    //        yError() << "[RobotController::configure] Unable to get the joint limits.";
    //        return false;
    //    }
    //    m_fingerIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buff, limits);

    m_jointsData.resize(Eigen::NoChange, m_numActuatedJoints);
    m_axesData.resize(Eigen::NoChange, m_numActuatedAxis);

    // estimaotr
    m_axisFeedbackEstimators = std::make_unique<Estimators>(m_numActuatedAxis);
    m_axisReferenceEstimators = std::make_unique<Estimators>(m_numActuatedAxis);

    m_jointFeedbackEstimators = std::make_unique<Estimators>(m_numActuatedJoints);
    m_jointExpectedEstimators = std::make_unique<Estimators>(m_numActuatedJoints);

    m_axisFeedbackEstimators->configure(config, name);
    m_axisReferenceEstimators->configure(config, name);

    m_jointFeedbackEstimators->configure(config, name);
    m_jointExpectedEstimators->configure(config, name);

    // linear regression
    m_linearRegressor = std::make_unique<LinearRegression>();

    // initialize the vectors and variables
    m_axisValueReferences.resize(m_numActuatedAxis, 0.0);
    m_axisValueFeedbacks.resize(m_numActuatedAxis, 0.0);

    m_jointValueReferences.resize(m_numActuatedJoints, 0.0);
    m_jointValueFeedbacks.resize(m_numActuatedJoints, 0.0);
    m_robotPrepared = false;
    m_estimatorsInitialized = false;

    // print info
    yInfo() << m_logPrefix << "number of all axis" << m_numAllAxis;
    yInfo() << m_logPrefix << "number of actuated axis" << m_numActuatedAxis;
    yInfo() << m_logPrefix << "number of all joints: " << m_numAllJoints;
    yInfo() << m_logPrefix << "number of actuated joints: " << m_numActuatedJoints;

    yInfo() << m_logPrefix << "axes and joints of the robot are coupled: " << m_axesJointsCoupled;
    yInfo() << m_logPrefix
            << "the coupling model between the robot axes and joints should be found: "
            << m_axesJointsCoupled;
    yInfo() << m_logPrefix << "robot controller exponential filter gain: " << m_kGain;

    //
    yInfo() << m_logPrefix << "configuration is done.";

    return true;
}

bool RobotController::setAxisReferences(const std::vector<double>& axisReferences)
{

    if (axisReferences.size() != m_numActuatedAxis)
    {
        yError() << m_logPrefix
                 << " the size of input vector and m_numActuatedAxis does not match.";
        return false;
    }
    m_axisValueReferences = axisReferences;

    return this->setAxisReferences(this->toEigen(m_axisValueReferences));
}

bool RobotController::setAxisReferences(const Eigen::VectorXd& axisReferences)
{
    if (axisReferences.size() != m_numActuatedAxis)
    {
        yError() << m_logPrefix
                 << " the size of input vector and m_numActuatedAxis does not match.";
        return false;
    }
    m_axisValueReferencesEigen = axisReferences;

    // axis feedbacks are already updated
    m_axisValueFeedbacksEigen = this->toEigen(m_axisValueFeedbacks);

    // exponential filter
    m_axisValueReferencesEigen
        = (1 - m_kGain) * m_axisValueFeedbacksEigen + m_kGain * m_axisValueReferencesEigen;

    this->toStd(m_axisValueReferencesEigen, m_axisValueReferences);

    return true;
}

bool RobotController::setJointReferences(const std::vector<double>& jointReferences)
{
    if (jointReferences.size() != m_numActuatedJoints)
    {
        yError() << m_logPrefix
                 << "the size of the "
                    "joint references and m_numActuatedJoints does not match.";
        return false;
    }
    m_jointValueReferences = jointReferences;
    m_jointValueReferencesEigen = this->toEigen(m_jointValueReferences);

    return true;
}

bool RobotController::computeControlSignals()
{
    m_axisValueReferencesEigen = m_controlCoeff * (m_jointValueReferencesEigen - m_Bias);

    return this->setAxisReferences(m_axisValueReferencesEigen);
}

void RobotController::getAxisValueReferences(std::vector<double>& axisReferences)
{
    axisReferences = m_axisValueReferences;
}

void RobotController::getJointReferences(std::vector<double>& fingerJointsReference)
{
    fingerJointsReference = m_jointValueReferences;
}

void RobotController::getJointExpectedValues(std::vector<double>& jointsValuesExpected)
{
    this->getAxisValueFeedbacks(m_axisValueFeedbacks);
    m_axisValueFeedbacksEigen = this->toEigen(m_axisValueFeedbacks);

    m_jointValuesExpectedEigen = m_A * m_axisValueFeedbacksEigen + m_Bias;
    this->toStd(m_jointValuesExpectedEigen, m_jointValuesExpected);
    jointsValuesExpected = m_jointValuesExpected;
}

bool RobotController::updateFeedback()
{
    if (!controlHelper()->getFeedback())
    {
        yInfo() << m_logPrefix
                << "unable the get the finger axis and joints values from the robot.";
        return false;
    }
    return true;
}

void RobotController::getAxisValueFeedbacks(std::vector<double>& axisValueFeedbacks)
{
    controlHelper()->axisFeedbacks(m_axisValueFeedbacks);
    axisValueFeedbacks = m_axisValueFeedbacks;
}

void RobotController::getAxisVelocityFeedbacks(std::vector<double>& fingerAxisVelocityFeedback)
{
    this->controlHelper()->jointEncodersSpeed(fingerAxisVelocityFeedback);
}

void RobotController::getJointValueFeedbacks(std::vector<double>& jointsValueFeedbacks)
{
    this->controlHelper()->allSensors(jointsValueFeedbacks);
}

void RobotController::getMotorCurrentFeedback(std::vector<double>& motorCurrentFeedback)
{
    this->controlHelper()->motorCurrents(motorCurrentFeedback);
}

void RobotController::getMotorCurrentReference(std::vector<double>& motorCurrentReference)
{
    this->controlHelper()->motorCurrentReference(motorCurrentReference);
}

void RobotController::getMotorPwmFeedback(std::vector<double>& motorPWMFeedback)
{
    this->controlHelper()->motorPwm(motorPWMFeedback);
}

void RobotController::getMotorPwmReference(std::vector<double>& motorPWMReference)
{
    this->controlHelper()->motorPwmReference(motorPWMReference);
}

void RobotController::getMotorPidOutputs(std::vector<double>& motorPidOutputs)
{
    this->controlHelper()->motorPidOutputs(motorPidOutputs);
}

// bool RobotController::LogDataToCalibrateRobotMotorsJointsCouplingRandom(
//    const bool generateRandomVelocity)
//{
//    if (!m_motorJointsCoupled)
//        return true;
//    if (!m_doCalibration)
//        return true;

//    if (m_fingerIntegrator == nullptr)
//    {
//        yError() << "[RobotController::setFingersVelocity] The integrator is not initialize "
//                    "please call configure() method";
//        return false;
//    }
//    /* Get the feedback and fill the matrices*/
//    // already getFeedback method has been called in upper level class
//    yarp::sig::Vector fingerJointsValues, fingerAxisValues;
//    getFingerAxisFeedback(fingerAxisValues);
//    getFingerJointsFeedback(fingerJointsValues);

//    const int noJoints = m_robotControlInterface->getNumberOfActuatedJoints();
//    const int noAxis = m_robotControlInterface->getNumberOfActuatedAxis();
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> fingerAxisValuesEigen;
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
//    fingerJointsValuesEigen; fingerAxisValuesEigen.resize(1, noAxis);
//    fingerJointsValuesEigen.resize(1, noJoints);
//    for (int i = 0; i < m_robotControlInterface->getNumberOfActuatedAxis(); i++)
//    {
//        fingerAxisValuesEigen(0, i) = fingerAxisValues(i);
//    }
//    for (int i = 0; i < m_robotControlInterface->getNumberOfActuatedJoints(); i++)
//    {
//        fingerJointsValuesEigen(0, i) = fingerJointsValues(i);
//    }

//    if (!push_back_row(m_motorsData, fingerAxisValuesEigen))
//        return false;

//    if (!push_back_row(m_jointsData, fingerJointsValuesEigen))
//        return false;

//    //    std::cout << "motor values:\n" << m_motorsData << std::endl;
//    //    std::cout << "joint values:\n" << m_jointsData << std::endl;
//    yarp::sig::Vector motorReference;
//    if (generateRandomVelocity)
//    {
//        yarp::sig::Vector motorVelocityReferenceIntegral;
//        yarp::sig::Matrix motorVelocitylimits;
//        getFingerAxisFeedback(motorReference);
//        m_robotControlInterface->getVelocityLimits(motorVelocitylimits);

//        for (int i = 0; i < m_robotControlInterface->getNumberOfActuatedAxis(); i++)
//        {
//            double randomVel
//                = -1.0 * motorVelocitylimits(i, 1)
//                  + (double(rand()) / double(RAND_MAX)) * 2.0 * motorVelocitylimits(i, 1);
//            yInfo() << "randvel" << randomVel << rand();
//            motorVelocityReference.push_back(randomVel);
//        }
//        yInfo() << "---------------> reference values 1: " << motorVelocitylimits(0, 0)
//                << motorVelocitylimits(0, 1) << motorVelocityReference(0);

//    } else
//    {
//    }
//    motorReference = m_fingerIntegrator->integrate(motorVelocityReference);
//    //    for (int i = 0; i < m_controlHelper->getActuatedDoFs(); i++)
//    //    {
//    //        motorReference += motorVelocityReferenceIntegral(i);
//    //    }

//    yInfo() << "---------------> reference values: " << motorReference(0);

//    setFingersAxisReference(motorReference);
//    move();

//    return true;
//}

bool RobotController::LogDataToCalibrateRobotAxesJointsCoupling(double time, int axisNumber)
{
    if (!m_axesJointsCoupled)
    {
        yInfo() << m_logPrefix << "axes and joints are not coupled, so returning.";
        return true;
    }
    if (!m_doCalibration)
    {
        yInfo() << m_logPrefix
                << "axes and joints are coupled, loading from the configuration file.";
        return true;
    }

    // get the feedback and fill the matrices
    // feedbacks are updated previously
    this->getAxisValueFeedbacks(m_axisValueFeedbacks);
    this->getJointValueFeedbacks(m_jointValueFeedbacks);

    m_axisValueFeedbacksEigen = this->toEigen(m_axisValueFeedbacks);
    m_jointValueFeedbacksEigen = this->toEigen(m_jointValueFeedbacks);

    if (!push_back_row(m_axesData, m_axisValueFeedbacksEigen))
    {
        yError() << m_logPrefix
                 << "cannot add new axes feedback values to the collected axes data .";
        return false;
    }
    if (!push_back_row(m_jointsData, m_jointValueFeedbacksEigen))
    {
        yError() << m_logPrefix
                 << "cannot add new joints feedback values to the collected joints data .";
        return false;
    }

    std::vector<double> minLimit, maxLimit;
    if (!m_robotControlInterface->getLimits(minLimit, maxLimit))
    {
        yError() << m_logPrefix << "cannot get the axis limits.";
        return false;
    }

    m_axisValueReferences = minLimit;

    m_axisValueReferences[axisNumber]
        = minLimit[axisNumber] + (maxLimit[axisNumber] - minLimit[axisNumber]) * sin(time);

    setAxisReferences(m_axisValueReferences);

    move();

    return true;
}

bool RobotController::trainCouplingMatrix()
{
    // adding bias term
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> motorsData, jointsData,
        A_Bias;

    motorsData.setOnes(m_axesData.rows(), m_axesData.cols() + 1);
    motorsData.block(0, 1, m_axesData.rows(), m_axesData.cols()) = m_axesData;
    A_Bias.resize(m_A.rows(), m_A.cols() + 1);
    jointsData = m_jointsData;

    m_linearRegressor->LearnOneShotMatrix(motorsData, jointsData, A_Bias);

    m_A = A_Bias.block(0, 1, m_A.rows(), m_A.cols());
    m_Bias = A_Bias.block(0, 0, m_Bias.rows(), 1);

    std::cout << m_logPrefix << "[axes-joints coupling] A (coupling) matrix:\n" << m_A << std::endl;
    std::cout << m_logPrefix << "[axes-joints coupling] B (bias) vector:\n" << m_Bias << std::endl;

    m_controlCoeff = ((m_A.transpose() * m_Q * m_A + m_R).inverse()) * m_A.transpose() * m_Q;

    std::cout << " QP control coefficient matrix:\n" << m_controlCoeff << std::endl;

    m_robotPrepared = true;

    return true;
}

bool RobotController::isRobotPrepared() const
{
    return m_robotPrepared;
}

bool RobotController::initializeEstimators()
{
    if (!isRobotPrepared())
    {
        yError() << m_logPrefix
                 << "the robot should be prepared before initialization of the estimator";
        return false;
    }

    getJointValueFeedbacks(m_jointValueFeedbacks);
    getJointExpectedValues(m_jointValuesExpected);

    m_axisFeedbackEstimators->initialize(m_axisValueFeedbacks);
    m_axisReferenceEstimators->initialize(m_axisValueReferences);
    m_jointFeedbackEstimators->initialize(m_jointValueFeedbacks);
    m_jointExpectedEstimators->initialize(m_jointValuesExpected);

    m_estimatorsInitialized = m_jointFeedbackEstimators->isInitialized()
                              && m_jointExpectedEstimators->isInitialized()
                              && m_axisFeedbackEstimators->isInitialized()
                              && m_axisReferenceEstimators->isInitialized();
    return true;
}

bool RobotController::areEstimatorsInitialized() const
{
    return m_estimatorsInitialized;
}

bool RobotController::estimateNextStates()
{
    yarp::sig::Vector axisReferenceVector, axisFeedbackVector;
    yarp::sig::Vector JointsExpectedVector, jointsFeedbackVector;

    if (m_axisFeedbackEstimators->isInitialized())
    {
        this->getAxisValueFeedbacks(m_axisValueFeedbacks);
        m_axisFeedbackEstimators->estimateNextState(m_axisValueFeedbacks);
    }
    if (m_axisReferenceEstimators->isInitialized())
    {
        this->getAxisValueReferences(m_axisValueReferences);
        m_axisReferenceEstimators->estimateNextState(m_axisValueReferences);
    }

    if (m_jointFeedbackEstimators->isInitialized())
    {
        this->getJointValueFeedbacks(m_jointValueFeedbacks);
        m_jointFeedbackEstimators->estimateNextState(m_jointValueFeedbacks);
    }

    if (m_jointExpectedEstimators->isInitialized())
    {
        this->getJointExpectedValues(m_jointValuesExpected);
        m_jointExpectedEstimators->estimateNextState(m_jointValuesExpected);
    }

    return true;
}

bool RobotController::getEstimatedMotorsState(
    std::vector<double>& feedbackAxisValuesEstimationKF,
    std::vector<double>& feedbackAxisVelocitiesEstimationKF,
    std::vector<double>& feedbackAxisAccelrationEstimationKF,
    Eigen::MatrixXd& feedbackAxisCovEstimationKF,
    std::vector<double>& referenceAxisValuesEstimationKF,
    std::vector<double>& referenceAxisVelocitiesEstimationKF,
    std::vector<double>& referenceAxisAccelrationEstimationKF,
    Eigen::MatrixXd& referenceAxisCovEstimationKF)
{

    if (m_axisFeedbackEstimators->isInitialized())
    {
        m_axisFeedbackEstimators->getInfo(feedbackAxisValuesEstimationKF,
                                          feedbackAxisVelocitiesEstimationKF,
                                          feedbackAxisAccelrationEstimationKF,
                                          feedbackAxisCovEstimationKF);
    }

    if (m_axisReferenceEstimators->isInitialized())
    {
        m_axisReferenceEstimators->getInfo(referenceAxisValuesEstimationKF,
                                           referenceAxisVelocitiesEstimationKF,
                                           referenceAxisAccelrationEstimationKF,
                                           referenceAxisCovEstimationKF);
    }

    return true;
}

bool RobotController::getEstimatedJointValuesKf(
    std::vector<double>& feedbackJointValuesEstimationKF,
    std::vector<double>& feedbackJointVelocitiesEstimationKF,
    std::vector<double>& feedbackJointAccelrationEstimationKF,
    Eigen::MatrixXd& feedbackJointCovEstimationKF,
    std::vector<double>& expectedJointValuesEstimationKF,
    std::vector<double>& expectedJointVelocitiesEstimationKF,
    std::vector<double>& expectedJointAccelrationEstimationKF,
    Eigen::MatrixXd& expectedJointCovEstimationKF)
{

    if (m_jointFeedbackEstimators->isInitialized())
    {
        m_jointFeedbackEstimators->getInfo(feedbackJointValuesEstimationKF,
                                           feedbackJointVelocitiesEstimationKF,
                                           feedbackJointAccelrationEstimationKF,
                                           feedbackJointCovEstimationKF);
    }

    if (m_jointExpectedEstimators->isInitialized())
    {
        m_jointExpectedEstimators->getInfo(expectedJointValuesEstimationKF,
                                           expectedJointVelocitiesEstimationKF,
                                           expectedJointAccelrationEstimationKF,
                                           expectedJointCovEstimationKF);
    }
    return true;
}

bool RobotController::getEstimatedJointValuesKf(
    std::vector<double>& feedbackJointValuesEstimationKF,
    std::vector<double>& expectedJointValuesEstimationKF)
{

    std::vector<double> feedbackJointVelocitiesEstimationKF, feedbackJointAccelrationEstimationKF;
    std::vector<double> expectedJointVelocitiesEstimationKF, expectedJointAccelrationEstimationKF;
    Eigen::MatrixXd feedbackJointCovEstimationKF, expectedJointCovEstimationKF;

    if (m_jointFeedbackEstimators->isInitialized())
    {
        m_jointFeedbackEstimators->getMotorValueInfo(feedbackJointValuesEstimationKF);
    }

    if (m_jointExpectedEstimators->isInitialized())
    {
        m_jointExpectedEstimators->getMotorValueInfo(expectedJointValuesEstimationKF);
    }
    return true;
}

bool RobotController::getCustomSetIndices(const std::vector<std::string>& allListName,
                                          const std::vector<std::string>& customListNames,
                                          const std::vector<double>& allListVector,
                                          std::vector<double>& customListVector)
{
    // check the sizes
    customListVector.clear();
    if (allListName.empty())
    {
        yError() << m_logPrefix << " allListName is empty.";
        return false;
    }
    if (allListVector.empty())
    {
        yError() << m_logPrefix << " allListVector is empty.";
        return false;
    }
    if (allListVector.size() != allListName.size())
    {
        yError() << m_logPrefix << " allListVector and allListName do not have similar sizes..";
        return false;
    }

    // find the custom vector
    for (unsigned i = 0; i < customListNames.size(); i++)
    {
        auto index = std::find(std::begin(allListName), std::end(allListName), customListNames[i]);

        if (index == std::end(allListName))
        {
            yError() << m_logPrefix << "cannot find a match for: " << customListNames[i]
                     << " , element: " << i << " in the allListName.";
            return false;
        }
        size_t element = index - allListName.begin();

        customListVector.push_back(allListVector[element]);
    }

    // one last final check
    if (customListNames.size() != customListVector.size())
    {
        yError() << m_logPrefix
                 << "customListName and customListVector "
                    "do not have similar sizes.";
        return false;
    }

    return true;
}

inline Eigen::Map<Eigen::VectorXd> RobotController::toEigen(std::vector<double>& vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

inline void RobotController::toStd(Eigen::VectorXd& vecEigen, std::vector<double>& vecStd)
{
    if (vecStd.size() != vecEigen.size())
    {
        vecStd.resize(vecEigen.size());
    }
    Eigen::VectorXd::Map(&vecStd[0], vecEigen.size()) = vecEigen;
}
