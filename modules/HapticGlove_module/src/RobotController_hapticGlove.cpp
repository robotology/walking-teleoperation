/**
 * @file RobotController_hapticGlove.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <RobotController_hapticGlove.hpp>
#include <Utils.hpp>

bool RobotController::configure(const yarp::os::Searchable& config, const std::string& name)
{
    m_robotControlInterface = std::make_unique<HapticGlove::RobotControlInterface>();
    if (!m_robotControlInterface->configure(config, name, false))
    {
        yError() << "[RobotController::configure] Unable to configure the control helper";
        return false;
    }

    size_t noActuatedAxis = m_robotControlInterface->getNumberOfActuatedAxis();

    double samplingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[RobotController::configure] Unable to find the sampling time";
        return false;
    }

    double fingerScaling;
    if (!YarpHelper::getDoubleFromSearchable(config, "fingersScaling", fingerScaling))
    {
        yError() << "[RobotController::configure] Initialization failed while reading "
                    "fingersScaling vector.";
        return false;
    }
    m_fingersScaling.resize(noActuatedAxis, fingerScaling);


    // check if the motors and joints are coupled
    m_motorJointsCoupled
        = config.check("motorsJointsCoupled", yarp::os::Value(0))
              .asBool(); /**<  true if the motors and joints of the robot are coupled*/
    yInfo() << "[RobotController::configure] motor and joints of the robot are coupled: "
            << m_motorJointsCoupled;

    // check if we need to do calibraition
    m_doCalibration = config.check("doCalibration", yarp::os::Value(0))
                          .asBool(); /**<  true if the motors and joints of the robot are coupled*/
    yInfo() << "[RobotController::configure] We have to do calibration to find the coupling "
               "between motors and joints "
            << m_motorJointsCoupled;

    size_t noAllJoints = m_robotControlInterface->getNumberOfActuatedJoints();
    size_t noActuatedJoints=m_robotControlInterface->getNumberOfActuatedJoints();
    if (m_motorJointsCoupled)
    {

        if (!m_doCalibration)
        {
            m_A.resize(noActuatedJoints, noActuatedAxis);
            yarp::sig::Vector A_vector;
            A_vector.resize(noActuatedJoints* noActuatedAxis);
            if (!YarpHelper::getYarpVectorFromSearchable(config, "CouplingMatrix", A_vector))
            {
                yError() << "[RobotController::configure] Initialization failed while reading "
                            "CouplingMatrix vector.";
                return false;
            }
            for (size_t i = 0; i < noAllJoints; i++)
            {
                for (size_t j = 0; j < noActuatedAxis; j++)
                {
                    size_t element = i * noActuatedAxis + j;
                    m_A(i, j) = A_vector(element);
                }
            }
        }
    } else
    {
        // in this case the mapping between the motors and joints are identity matrix
        m_A = Eigen::MatrixXd::Identity(noActuatedAxis, noActuatedAxis);
    }

    yInfo()<<"noAllJoints"<<noAllJoints;
    yInfo()<<"noActuatedAxis"<<noActuatedAxis;

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
    double q, r;
    if (!YarpHelper::getDoubleFromSearchable(config, "q_joint_motor", q))
    {
        yError() << "[RobotController::configure] Unable to find the q_joint_motor";
        return false;
    }
    if (!YarpHelper::getDoubleFromSearchable(config, "r_joint_motor", r))
    {
        yError() << "[RobotController::configure] Unable to find the r_joint_motor";
        return false;
    }

    m_Q = Eigen::MatrixXd::Identity(noActuatedJoints, noActuatedJoints)*q;
    m_R = Eigen::MatrixXd::Identity(noActuatedAxis, noActuatedAxis)*r;


    m_desiredMotorValue.resize(noActuatedAxis);
    m_desiredJointValue.resize(m_robotControlInterface->getNumberOfActuatedJoints());
    yarp::sig::Vector buff(noActuatedAxis, 0.0);

    updateFeedback();
    getFingerAxisFeedback(buff);
    yarp::sig::Matrix limits(noActuatedAxis, 2);
    if (!m_robotControlInterface->getLimits(limits))
    {
        yError() << "[RobotController::configure] Unable to get the joint limits.";
        return false;
    }
    m_fingerIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buff, limits);

    m_jointsData.resize(Eigen::NoChange, m_robotControlInterface->getNumberOfActuatedJoints());
    m_motorsData.resize(Eigen::NoChange, m_robotControlInterface->getNumberOfActuatedAxis());


    // Estimaotr

    m_robotMotorFeedbackEstimator= std::make_unique<RobotMotorsEstimation> (noActuatedAxis);
    m_robotMotorReferenceEstimator= std::make_unique<RobotMotorsEstimation> (noActuatedAxis);

    m_robotMotorFeedbackEstimator->configure(config, name);
    m_robotMotorReferenceEstimator->configure(config, name);

    std::cout<<"-- finish configuring m_robotMotorReferenceEstimator";



    m_robotPrepared=false;
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

    yarp::sig::Vector motorFeedbackValue= controlHelper()->jointEncoders();
    double k_gain= 0.9;
    for (unsigned i = 0; i < fingersReference.size(); i++)
    {
        m_desiredMotorValue(i) = fingersReference(i) * m_fingersScaling(i);
        m_desiredMotorValue(i) =  (1-k_gain)* motorFeedbackValue(i)+ k_gain* m_desiredMotorValue(i);
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
bool RobotController::setFingersJointReference(const yarp::sig::Vector& fingersReference)
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
        fingersJointsRef(i, 0) = fingersReference(i);
        m_desiredJointValue(i) = fingersReference(i);
    }
    fingersMotorRef = m_controlCoeff * fingersJointsRef;

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



bool RobotController::LogDataToCalibrateRobotMotorsJointsCouplingRandom(
    const bool generateRandomVelocity)
{
    yInfo() << "[RobotController::LogDataToCalibrateRobotMotorsJointsCoupling()]";
    if (!m_motorJointsCoupled)
        return true;
    if (!m_doCalibration)
        return true;

    if (m_fingerIntegrator == nullptr)
    {
        yError() << "[RobotController::setFingersVelocity] The integrator is not initialize "
                    "please call configure() method";
        return false;
    }
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

    if (!push_back_row(m_motorsData, fingerAxisValuesEigen))
        return false;

    if (!push_back_row(m_jointsData, fingerJointsValuesEigen))
        return false;

    //    std::cout << "motor values:\n" << m_motorsData << std::endl;
    //    std::cout << "joint values:\n" << m_jointsData << std::endl;
    yarp::sig::Vector motorReference;
    if (generateRandomVelocity)
    {
        yarp::sig::Vector motorVelocityReferenceIntegral;
        yarp::sig::Matrix motorVelocitylimits;
        getFingerAxisFeedback(motorReference);
        m_robotControlInterface->getVelocityLimits(motorVelocitylimits);

        for (int i = 0; i < m_robotControlInterface->getNumberOfActuatedAxis(); i++)
        {
            double randomVel
                = -1.0 * motorVelocitylimits(i, 1)
                  + (double(rand()) / double(RAND_MAX)) * 2.0 * motorVelocitylimits(i, 1);
            yInfo() << "randvel" << randomVel << rand();
            motorVelocityReference.push_back(randomVel);
        }
        yInfo() << "---------------> reference values 1: " << motorVelocitylimits(0, 0)
                << motorVelocitylimits(0, 1) << motorVelocityReference(0);

    } else
    {
    }
    motorReference = m_fingerIntegrator->integrate(motorVelocityReference);
    //    for (int i = 0; i < m_controlHelper->getActuatedDoFs(); i++)
    //    {
    //        motorReference += motorVelocityReferenceIntegral(i);
    //    }

    yInfo() << "---------------> reference values: " << motorReference(0);

    setFingersAxisReference(motorReference);
    move();

    return true;
}

bool RobotController::LogDataToCalibrateRobotMotorsJointsCouplingSin(double time, int axisNumber)
{
    yInfo() << "[RobotController::LogDataToCalibrateRobotMotorsJointsCoupling()]";
    if (!m_motorJointsCoupled)
        return true;
    if (!m_doCalibration)
        return true;

    if (m_fingerIntegrator == nullptr)
    {
        yError() << "[RobotController::setFingersVelocity] The integrator is not initialize "
                    "please call configure() method";
        return false;
    }
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

    if (!push_back_row(m_motorsData, fingerAxisValuesEigen))
        return false;

    if (!push_back_row(m_jointsData, fingerJointsValuesEigen))
        return false;

    //    std::cout << "motor values:\n" << m_motorsData << std::endl;
    //    std::cout << "joint values:\n" << m_jointsData << std::endl;
    yarp::sig::Vector motorReference;
    motorReference.resize(noAxis, 0.0); //0.0
//    motorReference(0)=0.5;
    motorReference(axisNumber) = M_PI_4 + M_PI_4 * sin(time);
    setFingersAxisReference(motorReference);
    move();
    yInfo() << "m_motorsData.size() : " << m_motorsData.rows() << m_motorsData.cols();
    yInfo() << "m_jointsData.size() : " << m_jointsData.rows() << m_jointsData.cols();

    return true;
}

bool RobotController::trainCouplingMatrix()
{
    yInfo() << "[RobotController::trainCouplingMatrix()]";

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> xT_x
        = (m_motorsData.transpose() * m_motorsData);
    yInfo() << "xT_x: " << xT_x.rows() << xT_x.cols();
    yInfo() << "xT_x.determinant(): " << xT_x.determinant();
    Eigen::FullPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> lu(
        xT_x);
    yInfo() << "lu.rank(): " << lu.rank();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> coeff
        = ((m_motorsData.transpose() * m_motorsData).inverse()) * m_motorsData.transpose(); // m X o

    for (int i = 0; i < m_robotControlInterface->getNumberOfActuatedJoints(); i++)
    {
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tetha_i
            = coeff * m_jointsData.col(i); // m X 1
        push_back_row(m_A, tetha_i.transpose());
    }
    yInfo() << "m_A matrix:" << m_A.rows() << m_A.cols();
    std::cout << "m_A matrix:\n" << m_A << std::endl;
    m_controlCoeff = ((m_A.transpose() * m_Q * m_A + m_R).inverse()) * m_A.transpose() * m_Q;
    yInfo() << "(m_A.transpose() * m_Q *m_A + m_R) matrix:"
            << (m_A.transpose() * m_Q * m_A + m_R).determinant();
    yInfo() << "m_controlCoeff matrix:" << m_controlCoeff.rows() << m_controlCoeff.cols();
    std::cout << "m_controlCoeff matrix:\n" << m_controlCoeff << std::endl;

    m_robotPrepared=true;
    return true;
}

bool RobotController::isRobotPrepared()
{
    return m_robotPrepared;
}

bool RobotController::initializeEstimator(){
    std::cout<<"RobotController::initializeEstimator() \n";
    yarp::sig::Vector axisReferenceVector, axisFeedbackVector ;
    getFingerAxisValueReference(axisReferenceVector);
    getFingerAxisFeedback(axisFeedbackVector);

    m_robotMotorFeedbackEstimator->initialize(axisFeedbackVector);
    m_robotMotorReferenceEstimator->initialize(axisReferenceVector);

    std::cout<<"RobotController::initializeEstimator() 2 \n";

    return true;
}

bool RobotController::estimateNextMotorsState(){

    yarp::sig::Vector axisReferenceVector, axisFeedbackVector ;
    getFingerAxisValueReference(axisReferenceVector);
    getFingerAxisFeedback(axisFeedbackVector);

    if(m_robotMotorFeedbackEstimator->isInitialized()){
        m_robotMotorFeedbackEstimator->estimateNextState(axisFeedbackVector);
    }

    if(m_robotMotorReferenceEstimator->isInitialized()){
        m_robotMotorReferenceEstimator->estimateNextState(axisReferenceVector);
    }

    return true;
}

bool RobotController::getEstimatedMotorsState(std::vector<double>& feedbackAxisValuesEstimationKF, std::vector<double>&  feedbackAxisVelocitiesEstimationKF, std::vector<double>&  feedbackAxisAccelrationEstimationKF,
                                              std::vector<double>& referenceAxisValuesEstimationKF, std::vector<double>&  referenceAxisVelocitiesEstimationKF, std::vector<double>&  referenceAxisAccelrationEstimationKF ){
    Eigen::VectorXd P;

    if(m_robotMotorFeedbackEstimator->isInitialized()){
        m_robotMotorFeedbackEstimator->getInfo(feedbackAxisValuesEstimationKF, feedbackAxisVelocitiesEstimationKF,   feedbackAxisAccelrationEstimationKF,  P );
    }

    if(m_robotMotorReferenceEstimator->isInitialized()){
        m_robotMotorReferenceEstimator->getInfo(referenceAxisValuesEstimationKF, referenceAxisVelocitiesEstimationKF,   referenceAxisAccelrationEstimationKF,  P );
    }

    return true;
}
