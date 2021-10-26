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

    // check if the motors and joints are coupled
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
        // in this case the mapping between the motors and joints are identity matrix
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

    // initialize the vectors
    m_desiredMotorValue.resize(m_numActuatedAxis);
    m_desiredJointValue.resize(m_numActuatedJoints);

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

bool RobotController::setFingersAxisReference(const yarp::sig::Vector& fingersReference)
{

    if (fingersReference.size() != m_desiredMotorValue.size())
    {
        yError() << "[RobotController::setFingersAxisReference] the size of the "
                    "fingersReference and m_desiredMotorValue does not match.";
        return false;
    }

    yarp::sig::Vector motorFeedbackValue = controlHelper()->jointEncoders();
    for (unsigned i = 0; i < fingersReference.size(); i++)
    {
        m_desiredMotorValue(i)
            = (1 - m_kGain) * motorFeedbackValue(i) + m_kGain * fingersReference(i);
    }

    //    if (m_fingerIntegrator == nullptr)
    //    {
    //        yError() << "[RobotController::setFingersVelocity] The integrator is not initialize
    //        "
    //                    "please call configure() method";
    //        return false;
    //    }
    //    // TOCHECK: maybe remove m_fingersScaling
    //    if (m_controlHelper->isVelocityControlUsed())
    //        m_desiredJointValue = fingersReference * m_fingersScaling; // velocity reference value
    //    else
    //        m_desiredJointValue = m_fingerIntegrator->integrate(fingersReference *
    //        m_fingersScaling);

    return true;
}
bool RobotController::setFingersJointReference(const std::vector<double>& fingersReference)
{
    if (fingersReference.size() != m_desiredJointValue.size())
    {
        yError() << "[RobotController::setFingersJointReference] the size of the "
                    "fingersReference and m_desiredJointValue does not match.";
        return false;
    }
    const int noJoints = m_robotControlInterface->getNumberOfActuatedJoints();
    const int noMotors = m_robotControlInterface->getNumberOfActuatedAxis();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> fingersJointsRef,
        fingersMotorRef;

    fingersJointsRef.resize(noJoints, 1);
    fingersMotorRef.resize(noMotors, 1);

    for (unsigned i = 0; i < noJoints; i++)
    {
        fingersJointsRef(i, 0) = fingersReference[i];
        m_desiredJointValue(i) = fingersReference[i];
    }
    fingersMotorRef = m_controlCoeff * (fingersJointsRef - m_Bias);

    yarp::sig::Vector fingersMotorsReference;
    fingersMotorsReference.resize(noMotors);

    for (int i = 0; i < noMotors; i++)
    {
        fingersMotorsReference(i) = fingersMotorRef(i, 0);
    }
    setFingersAxisReference(fingersMotorsReference);

    return true;
}

void RobotController::getFingerAxisValueReference(yarp::sig::Vector& fingerAxisReference)
{
    fingerAxisReference = m_desiredMotorValue;
}

void RobotController::getFingerAxisValueReference(std::vector<double>& fingerAxisReference)
{
    fingerAxisReference.clear();
    for (size_t i = 0; i < m_desiredMotorValue.size(); i++)
    {
        fingerAxisReference.push_back(m_desiredMotorValue(i));
    }
}

void RobotController::getFingerJointReference(yarp::sig::Vector& fingerJointsReference)
{
    fingerJointsReference = m_desiredJointValue;
}

void RobotController::getFingerJointReference(std::vector<double>& fingerJointsReference)
{
    fingerJointsReference.clear();
    for (size_t i = 0; i < m_desiredJointValue.size(); i++)
    {
        fingerJointsReference.push_back(m_desiredJointValue(i));
    }
}

void RobotController::getFingerJointExpectedValue(yarp::sig::Vector& fingerJointsExpectedValue)
{

    yarp::sig::Vector fingerAxisValues;
    getFingerAxisFeedback(fingerAxisValues);

    const int noJoints = m_robotControlInterface->getNumberOfActuatedJoints();
    const int noMotors = m_robotControlInterface->getNumberOfActuatedAxis();
    fingerJointsExpectedValue.resize(noJoints, 0.0);

    Eigen::VectorXd fingerJointsExpectedValueEigen, fingersMotorFeedbackEigen;

    fingerJointsExpectedValueEigen.resize(noJoints, 1);
    fingersMotorFeedbackEigen.resize(noMotors, 1);
    for (unsigned i = 0; i < noMotors; i++)
    {
        fingersMotorFeedbackEigen(i) = fingerAxisValues(i);
    }
    fingerJointsExpectedValueEigen = m_A * fingersMotorFeedbackEigen + m_Bias;

    for (unsigned i = 0; i < noJoints; i++)
    {
        fingerJointsExpectedValue(i) = fingerJointsExpectedValueEigen(i);
    }
}

void RobotController::getFingerJointExpectedValue(std::vector<double>& fingerJointsExpectedValue)
{
    yarp::sig::Vector fingerAxisValues;
    Eigen::VectorXd fingerJointsExpectedValueEigen, fingersMotorFeedbackEigen;

    getFingerAxisFeedback(fingerAxisValues);

    const int noJoints = m_robotControlInterface->getNumberOfActuatedJoints();
    const int noMotors = m_robotControlInterface->getNumberOfActuatedAxis();

    fingerJointsExpectedValue.resize(noJoints, 0.0);
    fingerJointsExpectedValueEigen.resize(noJoints, 1);
    fingersMotorFeedbackEigen.resize(noMotors, 1);

    for (unsigned i = 0; i < noMotors; i++)
    {
        fingersMotorFeedbackEigen(i) = fingerAxisValues(i);
    }
    fingerJointsExpectedValueEigen = m_A * fingersMotorFeedbackEigen + m_Bias;

    for (unsigned i = 0; i < noJoints; i++)
    {
        fingerJointsExpectedValue[i] = fingerJointsExpectedValueEigen(i);
    }
}

bool RobotController::updateFeedback()
{
    if (!controlHelper()->getFeedback())
    {
        yInfo() << "[RobotController::getFingerAxisValues] Unable the get the finger axis and "
                   "joints values "
                   "from the robot.";
        return false;
    }
    return true;
}

void RobotController::getFingerAxisFeedback(yarp::sig::Vector& fingerAxisValues)
{
    fingerAxisValues.clear();
    fingerAxisValues = controlHelper()->jointEncoders();
}

void RobotController::getFingerAxisFeedback(std::vector<double>& fingerAxisValues)
{
    fingerAxisValues.clear();
    yarp::sig::Vector Temp = controlHelper()->jointEncoders();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        fingerAxisValues.push_back(Temp(i));
    }
}
void RobotController::getFingerAxisVelocityFeedback(yarp::sig::Vector& fingerAxisVelocityFeedback)
{
    fingerAxisVelocityFeedback.clear();
    fingerAxisVelocityFeedback = controlHelper()->jointEncodersSpeed();
}

void RobotController::getFingerAxisVelocityFeedback(std::vector<double>& fingerAxisVelocityFeedback)
{
    fingerAxisVelocityFeedback.clear();
    yarp::sig::Vector Temp = controlHelper()->jointEncodersSpeed();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        fingerAxisVelocityFeedback.push_back(Temp(i));
    }
}
void RobotController::getFingerJointsFeedback(yarp::sig::Vector& fingerJointsValues)
{
    fingerJointsValues.clear();
    fingerJointsValues = controlHelper()->allSensors();
}

void RobotController::getFingerJointsFeedback(std::vector<double>& fingerJointsValues)
{
    fingerJointsValues.clear();
    yarp::sig::Vector Temp = controlHelper()->allSensors();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        fingerJointsValues.push_back(Temp(i));
    }
}

void RobotController::getMotorCurrentFeedback(yarp::sig::Vector& motorCurrentFeedback)
{
    motorCurrentFeedback.clear();
    motorCurrentFeedback = controlHelper()->motorCurrents();
}

void RobotController::getMotorCurrentFeedback(std::vector<double>& motorCurrentFeedback)
{
    motorCurrentFeedback.clear();
    yarp::sig::Vector Temp = controlHelper()->motorCurrents();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        motorCurrentFeedback.push_back(Temp(i));
    }
}

void RobotController::getMotorCurrentReference(yarp::sig::Vector& motorCurrentReference)
{
    motorCurrentReference.clear();
    motorCurrentReference = controlHelper()->motorCurrentReference();
}

void RobotController::getMotorCurrentReference(std::vector<double>& motorCurrentReference)
{
    motorCurrentReference.clear();
    yarp::sig::Vector Temp = controlHelper()->motorCurrentReference();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        motorCurrentReference.push_back(Temp(i));
    }
}

void RobotController::getMotorPwmFeedback(std::vector<double>& motorPWMFeedback)
{
    motorPWMFeedback.clear();
    yarp::sig::Vector Temp = controlHelper()->motorPwm();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        motorPWMFeedback.push_back(Temp(i));
    }
}

void RobotController::getMotorPwmFeedback(yarp::sig::Vector& motorPWMFeedback)
{
    motorPWMFeedback.clear();
    motorPWMFeedback = controlHelper()->motorPwm();
}

void RobotController::getMotorPwmReference(std::vector<double>& motorPWMReference)
{
    motorPWMReference.clear();
    yarp::sig::Vector Temp = controlHelper()->motorPwmReference();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        motorPWMReference.push_back(Temp(i));
    }
}

void RobotController::getMotorPidOutputs(std::vector<double>& motorPidOutputs)
{
    motorPidOutputs.clear();
    yarp::sig::Vector Temp = controlHelper()->motorPidOutputs();
    for (size_t i = 0; i < Temp.size(); i++)
    {
        motorPidOutputs.push_back(Temp(i));
    }
}

void RobotController::getMotorPidOutputs(yarp::sig::Vector& motorPidOutputs)
{
    motorPidOutputs.clear();
    motorPidOutputs = controlHelper()->motorPidOutputs();
}

void RobotController::getMotorPwmReference(yarp::sig::Vector& motorPWMReference)
{
    motorPWMReference.clear();
    motorPWMReference = controlHelper()->motorPwmReference();
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

bool RobotController::LogDataToCalibrateRobotMotorsJointsCouplingSin(double time, int axisNumber)
{
    yInfo() << "[RobotController::LogDataToCalibrateRobotMotorsJointsCoupling()]";
    if (!m_axesJointsCoupled)
        return true;
    if (!m_doCalibration)
        return true;

    /* Get the feedback and fill the matrices*/
    // already getFeedback method has been called in upper level class
    yarp::sig::Vector fingerJointsValues, fingerAxisValues;
    getFingerAxisFeedback(fingerAxisValues);
    getFingerJointsFeedback(fingerJointsValues);

    const int noJoints = m_robotControlInterface->getNumberOfActuatedJoints();
    const int noAxis = m_robotControlInterface->getNumberOfActuatedAxis();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> fingerAxisValuesEigen;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> fingerJointsValuesEigen;
    fingerAxisValuesEigen.resize(1, noAxis);
    fingerJointsValuesEigen.resize(1, noJoints);
    for (int i = 0; i < m_robotControlInterface->getNumberOfActuatedAxis(); i++)
    {
        fingerAxisValuesEigen(0, i) = fingerAxisValues(i);
    }
    for (int i = 0; i < m_robotControlInterface->getNumberOfActuatedJoints(); i++)
    {
        fingerJointsValuesEigen(0, i) = fingerJointsValues(i);
    }

    if (!push_back_row(m_axesData, fingerAxisValuesEigen))
        return false;

    if (!push_back_row(m_jointsData, fingerJointsValuesEigen))
        return false;

    yarp::sig::Vector motorReference;
    motorReference.resize(noAxis, 0.0); // 0.0
    yarp::sig::Matrix limits;
    if (!m_robotControlInterface->getLimits(limits))
    {
        yError() << "[RobotController::LogDataToCalibrateRobotMotorsJointsCouplingSin] Cannot get "
                    "the axis limits. ";
    }
    for (size_t i = 0; i < noAxis; i++)
    {
        motorReference(i) = limits(i, 0);
    }
    motorReference(axisNumber)
        = limits(axisNumber, 0) + (limits(axisNumber, 1) - limits(axisNumber, 0)) * sin(time);

    setFingersAxisReference(motorReference);
    move();
    yInfo() << "m_motorsData.size() : " << m_axesData.rows() << m_axesData.cols();
    yInfo() << "m_jointsData.size() : " << m_jointsData.rows() << m_jointsData.cols();

    return true;
}

bool RobotController::trainCouplingMatrix()
{
    yInfo() << "[RobotController::trainCouplingMatrix()]";

    // adding bias term
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> motorsData, jointsData,
        A_Bias;
    yInfo() << "(Bias+ A) matrix size:" << A_Bias.rows() << A_Bias.cols();
    motorsData.setOnes(m_axesData.rows(), m_axesData.cols() + 1);
    motorsData.block(0, 1, m_axesData.rows(), m_axesData.cols()) = m_axesData;
    A_Bias.resize(m_A.rows(), m_A.cols() + 1);
    jointsData = m_jointsData;
    yInfo() << "(Bias+ A) matrix size:" << A_Bias.rows() << A_Bias.cols();

    m_linearRegressor->LearnOneShotMatrix(motorsData, jointsData, A_Bias);

    m_A = A_Bias.block(0, 1, m_A.rows(), m_A.cols());
    m_Bias = A_Bias.block(0, 0, m_Bias.rows(), 1);

    yInfo() << "(Bias+ A) matrix size:" << A_Bias.rows() << A_Bias.cols();
    yInfo() << "(m_A) matrix size:" << m_A.rows() << m_A.cols();
    yInfo() << "(m_Bias) matrix size:" << m_Bias.rows() << m_Bias.cols();

    std::cout << "(Bias+ A) matrix:\n" << A_Bias << std::endl;
    std::cout << "(A) matrix:\n" << m_A << std::endl;
    std::cout << "(Bias) matrix:\n" << m_Bias << std::endl;

    //    m_linearRegressor->LearnOneShotMatrix(m_motorsData, m_jointsData, m_A);

    //    yInfo() << "m_A matrix:" << m_A.rows() << m_A.cols();
    //    std::cout << "m_A matrix:\n" << m_A << std::endl;

    m_controlCoeff = ((m_A.transpose() * m_Q * m_A + m_R).inverse()) * m_A.transpose() * m_Q;

    yInfo() << "m_controlCoeff matrix:" << m_controlCoeff.rows() << m_controlCoeff.cols();
    std::cout << " control coefficient matrix:\n" << m_controlCoeff << std::endl;

    m_robotPrepared = true;
    return true;
}

bool RobotController::isRobotPrepared() const
{
    return m_robotPrepared;
}

bool RobotController::initializeEstimators()
{
    std::cout << "RobotController::initializeEstimator() \n";
    if (!isRobotPrepared())
    {
        yError() << "[RobotController::initializeEstimators()] the robot should be prepared before "
                    "initialization "
                    "of the estimator";
        return false;
    }
    yarp::sig::Vector axisReferenceVector, axisFeedbackVector;
    getFingerAxisValueReference(axisReferenceVector);
    getFingerAxisFeedback(axisFeedbackVector);
    m_axisFeedbackEstimators->initialize(axisFeedbackVector);
    m_axisReferenceEstimators->initialize(axisReferenceVector);

    yarp::sig::Vector jointsExpectedVector, jointsFeedbackVector;
    getFingerJointsFeedback(jointsFeedbackVector);
    getFingerJointExpectedValue(jointsExpectedVector);

    m_jointFeedbackEstimators->initialize(jointsFeedbackVector);
    m_jointExpectedEstimators->initialize(jointsExpectedVector);

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
        getFingerAxisFeedback(axisFeedbackVector);
        m_axisFeedbackEstimators->estimateNextState(axisFeedbackVector);
    }
    if (m_axisReferenceEstimators->isInitialized())
    {
        getFingerAxisValueReference(axisReferenceVector);

        m_axisReferenceEstimators->estimateNextState(axisReferenceVector);
    }

    if (m_jointFeedbackEstimators->isInitialized())
    {
        getFingerJointsFeedback(jointsFeedbackVector);
        m_jointFeedbackEstimators->estimateNextState(jointsFeedbackVector);
    }

    if (m_jointExpectedEstimators->isInitialized())
    {
        getFingerJointExpectedValue(JointsExpectedVector);
        m_jointExpectedEstimators->estimateNextState(JointsExpectedVector);
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

bool RobotController::getEstimatedJointState(
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

bool RobotController::getEstimatedJointState(std::vector<double>& feedbackJointValuesEstimationKF,
                                             std::vector<double>& expectedJointValuesEstimationKF)
{

    std::vector<double> feedbackJointVelocitiesEstimationKF, feedbackJointAccelrationEstimationKF;
    std::vector<double> expectedJointVelocitiesEstimationKF, expectedJointAccelrationEstimationKF;
    Eigen::MatrixXd feedbackJointCovEstimationKF, expectedJointCovEstimationKF;

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
