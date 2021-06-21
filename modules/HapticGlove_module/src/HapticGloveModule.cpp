/**
 * @file HapticGloveModule.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <HapticGloveModule.hpp>
#include <Utils.hpp>

#include <Eigen/Dense>
#include <thread>

HapticGloveModule::HapticGloveModule(){};

HapticGloveModule::~HapticGloveModule(){};

bool HapticGloveModule::configure(yarp::os::ResourceFinder& rf)
{

    yarp::os::Value* value;

    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[HapticGloveModule::configure] Empty configuration for the HapticGloveModule "
                    "application.";
        return false;
    }
    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asDouble();

    // check if move the robot
    m_moveRobot = generalOptions.check("enableMoveRobot", yarp::os::Value(1)).asBool();
    yInfo() << "[HapticGloveModule::configure] move the robot: " << m_moveRobot;

    // check if log the data
    m_enableLogger = generalOptions.check("enableLogger", yarp::os::Value(0)).asBool();
    yInfo() << "[HapticGloveModule::configure] enable the logger: " << m_enableLogger;

    // check time period
    m_calibrationTimePeriod = generalOptions.check("calibrationTimePeriod", yarp::os::Value(10.0)).asDouble();
    yInfo() << "[HapticGloveModule::configure] calibration time period: " << m_calibrationTimePeriod;

    double smoothingTime = generalOptions.check("smoothingTime", yarp::os::Value(1.0)).asDouble();
    yInfo()<<"[HapticGloveModule::configure] smoothingTime :"<<smoothingTime ;

    // robot name:
    m_robot = generalOptions.check("robot", yarp::os::Value("icubSim")).asString();

    // check if find the user motion range
    m_getHumanMotionRange = generalOptions.check("getHumanMotionRange", yarp::os::Value(0)).asBool();


    // check which hand to use
    m_useLeftHand = generalOptions.check("useLeftHand", yarp::os::Value(1)).asBool();
    m_useRightHand = generalOptions.check("useRightHand", yarp::os::Value(1)).asBool();
    yInfo() << "[HapticGloveModule::configure] use the left hand: " << m_useLeftHand;
    yInfo() << "[HapticGloveModule::configure] use the right hand: " << m_useRightHand;

    // configure fingers retargeting
    if (m_useLeftHand)
    {
        // ########
        // ######## initialize the robot control class of the left hand
        // ########
        m_robotLeftHand = std::make_unique<RobotController>();
        yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
        leftFingersOptions.append(generalOptions);
        if (!m_robotLeftHand->configure(leftFingersOptions, getName()))
        {
            yError() << "[HapticGloveModule::configure] Unable to initialize the left fingers "
                        "retargeting.";
            return false;
        }

        // ########
        // ######## intialize the glove control helper of the left hand
        // ########
        m_gloveLeftHand = std::make_unique<HapticGlove::GloveControlHelper>();
        if (!m_gloveLeftHand->configure(leftFingersOptions, getName(), false))
        {
            yError()
                    << "[HapticGloveModule::configure] Unable to initialize the right glove control "
                       "helper.";
            return false;
        }

        //TODEL-->START
        /*
        m_leftTotalGain.resize(m_robotLeftHand->controlHelper()->getActuatedDoFs(), 0.0);
        m_leftVelocityGain.resize(m_robotLeftHand->controlHelper()->getActuatedDoFs(), 0.0);
        m_leftBuzzMotorsGain.resize(m_gloveLeftHand->getNoOfBuzzMotors(), 0.0);
        if(!YarpHelper::getYarpVectorFromSearchable(leftFingersOptions, "K_GainTotal", m_leftTotalGain))
        {
            yError() << "[HapticGloveModule::configure] Initialization failed while reading K_GainTotal vector of the left hand.";
            return false;
        }
        if(!YarpHelper::getYarpVectorFromSearchable(leftFingersOptions, "K_GainVelocity", m_leftVelocityGain))
        {
            yError() << "[HapticGloveModule::configure] Initialization failed while reading K_GainVelocity vector of the left hand.";
            return false;
        }

        if(!YarpHelper::getYarpVectorFromSearchable(leftFingersOptions, "K_GainBuzzMotors", m_leftBuzzMotorsGain))
        {
            yError() << "[HapticGloveModule::configure] Initialization failed while reading K_GainBuzzMotors vector of the left hand.";
            return false;
        }
        */
        //TODEL-->END

        // ########
        // ######## Retargeting configuration
        // ########

        size_t noRobotAllAxis = m_robotLeftHand->controlHelper()->getNumberOfAllAxis();
        size_t noRobotActuatedAxis = m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis();
        std::vector<std::string> robotActuatedJointNameList;
        m_robotLeftHand->controlHelper()->getActuatedJointNameList(robotActuatedJointNameList);
        std::vector<std::string> robotActuatedAxisNameList;
        m_robotLeftHand->controlHelper()->getActuatedAxisNameList(robotActuatedAxisNameList);

        size_t noBuzzMotors = m_gloveLeftHand->getNoOfBuzzMotors();
        std::vector<std::string> humanJointNameList;
        m_gloveLeftHand->getHumanJointsList(humanJointNameList);

        m_retargetingLeftHand= std::make_unique<Retargeting>(noRobotAllAxis, noRobotActuatedAxis, noBuzzMotors, robotActuatedJointNameList,
                                                             robotActuatedAxisNameList, humanJointNameList );
        if(!m_retargetingLeftHand->configure(leftFingersOptions, getName()))
        {
            yError()<<"[HapticGloveModule::configure] m_retargetingLeftHand->configure returns false";
            return false;
        }


    }
    if (m_useRightHand)
    {
        // ########
        // ######## initialize the robot control class of the right hand
        // ########
        m_robotRightHand = std::make_unique<RobotController>();
        yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
        rightFingersOptions.append(generalOptions);
        if (!m_robotRightHand->configure(rightFingersOptions, getName()))
        {
            yError() << "[HapticGloveModule::configure] Unable to initialize the right fingers "
                        "retargeting.";
            return false;
        }

        // ########
        // ######## intialize the glove control helper of the right hand
        // ########
        m_gloveRightHand = std::make_unique<HapticGlove::GloveControlHelper>();
        if (!m_gloveRightHand->configure(rightFingersOptions, getName(), true))
        {
            yError()
                    << "[HapticGloveModule::configure] Unable to initialize the right glove control "
                       "helper.";
            return false;
        }

        //TODEL-->START
        /*
        m_rightTotalGain.resize(m_robotRightHand->controlHelper()->getActuatedDoFs(), 0.0);
        m_rightVelocityGain.resize(m_robotRightHand->controlHelper()->getActuatedDoFs(), 0.0);
        m_rightBuzzMotorsGain.resize(m_gloveRightHand->getNoOfBuzzMotors(), 0.0);
        if(!YarpHelper::getYarpVectorFromSearchable(rightFingersOptions, "K_GainTotal", m_rightTotalGain))
        {
            yError() << "[HapticGloveModule::configure] Initialization failed while reading K_GainTotal vector of the right hand.";
            return false;
        }
        if(!YarpHelper::getYarpVectorFromSearchable(rightFingersOptions, "K_GainVelocity", m_rightVelocityGain))
        {
            yError() << "[HapticGloveModule::configure] Initialization failed while reading K_GainVelocity vector of the right hand.";
            return false;
        }

        if(!YarpHelper::getYarpVectorFromSearchable(rightFingersOptions, "K_GainBuzzMotors", m_rightBuzzMotorsGain))
        {
            yError() << "[HapticGloveModule::configure] Initialization failed while reading K_GainBuzzMotors vector of the right hand.";
            return false;
        }
        */
        //TODEL-->END


        // ########
        // ######## Retargeting configuration
        // ########

        size_t noRobotAllAxis = m_robotRightHand->controlHelper()->getNumberOfAllAxis();
        size_t noRobotActuatedAxis = m_robotRightHand->controlHelper()->getNumberOfActuatedAxis();
        std::vector<std::string> robotActuatedJointNameList;
        m_robotRightHand->controlHelper()->getActuatedJointNameList(robotActuatedJointNameList);
        std::vector<std::string> robotActuatedAxisNameList;
        m_robotRightHand->controlHelper()->getActuatedAxisNameList(robotActuatedAxisNameList);

        size_t noBuzzMotors = m_gloveRightHand->getNoOfBuzzMotors();
        std::vector<std::string> humanJointNameList;
        m_gloveRightHand->getHumanJointsList(humanJointNameList);

        m_retargetingRightHand= std::make_unique<Retargeting>(noRobotAllAxis,noRobotActuatedAxis, noBuzzMotors, robotActuatedJointNameList,
                                                              robotActuatedAxisNameList, humanJointNameList );

        if(!m_retargetingRightHand->configure(rightFingersOptions, getName()))
        {
            yError()<<"[HapticGloveModule::configure] m_retargetingRightHand->configure returns false";
            return false;
        }
    }
    std::cerr << "conf 05 \n";

    if (m_useLeftHand)
    {
        //ROBOT

         m_icubLeftFingerJointsReference.resize(
                     m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints(), 0.0);
         m_icubLeftFingerJointsFeedback.resize(
                     m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints(), 0.0);

         m_icubLeftFingerAxisValueReference.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
         m_icubLeftFingerAxisValueFeedback.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);

         m_icubLeftFingerAxisVelocityReference.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(),0.0);
         m_icubLeftFingerAxisVelocityFeedback.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(),0.0);

         m_icubLeftFingerAxisValueError.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(),0.0);
         m_icubLeftFingerAxisValueErrorSmoothed.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(),0.0);
         m_icubLeftFingerAxisVelocityError.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(),0.0);
         m_icubLeftFingerAxisVelocityErrorSmoothed.resize(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(),0.0);

         yarp::sig::Vector buff(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
         m_leftAxisValueErrorSmoother
                 = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), m_dT, smoothingTime);
         m_leftAxisValueErrorSmoother->init(buff);

         m_leftAxisVelocityErrorSmoother
                 = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), m_dT, smoothingTime);
         m_leftAxisVelocityErrorSmoother->init(buff);

        // HUMAN
        m_gloveLeftBuzzMotorReference.resize(m_gloveLeftHand->getNoOfBuzzMotors(), 0.0);
        m_gloveLeftForceFeedbackReference.resize(m_gloveLeftHand->getNoOfForceFeedback(), 0.0);
    }
    std::cerr << "conf 06 \n";
    if (m_useRightHand)
    {

        //ROBOT
        m_icubRightFingerJointsReference.resize(
                    m_robotRightHand->controlHelper()->getNumberOfActuatedJoints(), 0.0);
        m_icubRightFingerJointsFeedback.resize(
                    m_robotRightHand->controlHelper()->getNumberOfActuatedJoints(),0.0);

        m_icubRightFingerAxisValueReference.resize(
                    m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
        m_icubRightFingerAxisValueFeedback.resize(
                    m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);

        m_icubRightFingerAxisVelocityReference.resize(
                    m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
        m_icubRightFingerAxisVelocityFeedback.resize(
                    m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);

        m_icubRightFingerAxisValueError.resize(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(),0.0);
        m_icubRightFingerAxisValueErrorSmoothed.resize(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(),0.0);
        m_icubRightFingerAxisVelocityError.resize(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(),0.0);;
        m_icubRightFingerAxisVelocityErrorSmoothed.resize(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(),0.0);

        yarp::sig::Vector buff(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
        m_rightAxisValueErrorSmoother
                = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), m_dT, smoothingTime);
        m_rightAxisValueErrorSmoother->init(buff);
        m_rightAxisVelocityErrorSmoother
                = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), m_dT, smoothingTime);
        m_rightAxisVelocityErrorSmoother->init(buff);

        //HUMAN
        m_gloveRightBuzzMotorReference.resize(m_gloveRightHand->getNoOfBuzzMotors(), 0.0);
        m_gloveRightForceFeedbackReference.resize(m_gloveRightHand->getNoOfForceFeedback(), 0.0);
    }

    m_timePreparation = 0.0;
    m_timeNow = 0.0;
    m_timeConfigurationEnding = 0.0;
    std::cerr << "conf 07 \n";

    m_velocity_threshold_transient =3.5;
    m_Value_error_threshold_transient=0.1;




    // open the logger only if all the vecotos sizes are clear.
    if (m_enableLogger)
    {
        if (!openLogger())
        {
            yError() << "[HapticGloveModule::configure] Unable to open the logger";
            return false;
        }
    }
    std::cerr << "conf 08 \n";
    yInfo()<<"[HapticGloveModule::configure] Configuration is done. ";
    m_state = HapticGloveFSM::Configured;

    return true;
}

bool calibrateRobotMotorsJointsCoupling()
{

    return true;
}
double HapticGloveModule::getPeriod()
{
    return m_dT;
}

bool HapticGloveModule::close()
{
#ifdef ENABLE_LOGGER
    if (m_enableLogger)
    {
        m_logger->flush_available_data();
    }
#endif
    // m_logger.reset();
    if (m_useLeftHand)
    {
        m_gloveLeftHand->stopFeedback();
        m_gloveLeftHand->turnOffBuzzMotors();
        m_gloveLeftHand->turnForceFeedback();

    }

    if (m_useRightHand)
    {
        m_gloveRightHand->stopFeedback();
        m_gloveRightHand->turnOffBuzzMotors();
        m_gloveRightHand->turnForceFeedback();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 10ms.
    return true;
}

bool HapticGloveModule::evaluateRobotFingersReferences()
{
    return true;
}

bool HapticGloveModule::getFeedbacks()
{
    // 1- get the joint ref values from the haptic glove

    // 2- get the feedback from the iCub hands [force (holding force) and tactile
    // information]

    if (m_useLeftHand)
    {
        double time0= yarp::os::Time::now();

        // get feedback from the robot left hand values
        if (!m_robotLeftHand->updateFeedback())
        {
            yError()
                    << "[HapticGloveModule::getFeedbacks()] unable to update the feedback values of "
                       "the left hand fingers.";
        }
        double time1= yarp::os::Time::now();
        yInfo()<<"time D1:"<<time1-time0;


        m_robotLeftHand->getFingerAxisFeedback(m_icubLeftFingerAxisValueFeedback);
        double time2= yarp::os::Time::now();
        yInfo()<<"time D2:"<<time2-time1;

        m_robotLeftHand->getFingerAxisValueReference(m_icubLeftFingerAxisValueReference);
        double time3= yarp::os::Time::now();
        yInfo()<<"time D3:"<<time3-time2;

        m_robotLeftHand->getFingerAxisVelocityFeedback( m_icubLeftFingerAxisVelocityFeedback);
        double time4= yarp::os::Time::now();
        yInfo()<<"time D4:"<<time4-time3;

        m_robotLeftHand->getFingerJointsFeedback(m_icubLeftFingerJointsFeedback);
        double time5= yarp::os::Time::now();
        yInfo()<<"time D5:"<<time5-time4;


//        yInfo() << "left fingers axis: " << m_icubLeftFingerAxisValueFeedback.toString();
//        yInfo() << "left fingers joints: " << m_icubLeftFingerJointsFeedback.toString();
        m_robotLeftHand->estimateNextStates();
        double time6= yarp::os::Time::now();
        yInfo()<<"time D6:"<<time6-time5;
        yInfo()<<"total feedback time:"<<time6-time0;

    }

    if (m_useRightHand)
    {
        // get feedback from the robot right hand values
        if (!m_robotRightHand->updateFeedback())
        {
            yError()
                    << "[HapticGloveModule::getFeedbacks()] unable to update the feedback values of "
                       "the right hand fingers.";
        }
        m_robotRightHand->getFingerAxisFeedback(m_icubRightFingerAxisValueFeedback);
        m_robotRightHand->getFingerAxisValueReference(m_icubRightFingerAxisValueReference);
        m_robotRightHand->getFingerAxisVelocityFeedback( m_icubRightFingerAxisVelocityFeedback);

        m_robotRightHand->getFingerJointsFeedback(m_icubRightFingerJointsFeedback);
        yInfo() << "right fingers axis feedback: " << m_icubRightFingerAxisValueFeedback.toString();
        yInfo() << "right fingers axis reference: " << m_icubRightFingerAxisValueReference.toString();
        yInfo() << "right fingers joints: " << m_icubRightFingerJointsFeedback.toString();

        m_robotRightHand->estimateNextStates();

    }
    yInfo()<<"getFeedback success.";

    return true;
}

bool HapticGloveModule::updateModule()
{
    double time0= yarp::os::Time::now();
    yInfo() << "********************************* [HapticGloveModule::updateModule()] "
               "*********************************";
    if (!getFeedbacks())
    {
        yError() << "[HapticGloveModule::updateModule] Unable to get the feedback";
        return false;
    }
    double time1=yarp::os::Time::now();
    yInfo()<<"[updateModule] time feedback:"<<time1-time0;

    if (m_state == HapticGloveFSM::Running)
    {
        yInfo() << " >>> STATUS: Running";

        m_timeNow = yarp::os::Time::now();
        if(false)
        {
            if (m_useLeftHand)
            {
                // 1- Compute the reference values for the iCub hand joint fingers
                const unsigned noLeftFingersAxis
                        = m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis();
                const unsigned noLeftFingersJoints
                        = m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints();

                for (unsigned i = 0; i < noLeftFingersJoints; i++)
                {
                    //            m_icubLeftFingerAxisReference(i)
                    m_icubLeftFingerJointsReference(i)
                            = M_PI_4 + M_PI_4 * sin((m_timeNow - m_timePreparation) - M_PI_2);
                    //            m_icubRightFingerAxisReference(i) = m_icubLeftFingerAxisReference(i);
                    //            m_icubLeftFingerJointsReference(i) = m_icubLeftFingerAxisReference(i);
                }
            }
            double tmp_buzzVal;
            if ((m_timeNow - m_timePreparation) < (50 * M_PI))
            {
                yInfo() << "give buzz ref value higher than zero, time: "
                        << m_timeNow - m_timePreparation;
                tmp_buzzVal = 85;
                // 0 + 20.0 * sin((m_timeNow - m_timePreparationStarting) / 4.0 - M_PI_2);
            } else
            {
                yInfo() << "give buzz ref value equal to zero, time: "
                        << m_timeNow - m_timePreparation;
                tmp_buzzVal = 0.0;
            }
            for (int i = 0; i < 5; i++)
                m_gloveRightBuzzMotorReference(i) = tmp_buzzVal;
            // m_gloveRightBuzzMotorReference(1) = tmp_buzzVal;

            // m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);

            // m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);
            // m_gloveRightHand->setFingersForceReference(m_gloveRightBuzzMotorReference);
        }
        if (false)
        {
            if (m_timeNow - m_timePreparation < 10.0)
            {
                // m_timePreparationStarting = m_timeNow;
                yInfo() << "++++++++++++++++++++++++ time: "
                        << m_timeNow - m_timePreparation << " give force feedback command";
                //  m_gloveRightHand->setPalmFeedbackThumper(0);
                m_gloveRightHand->setFingersForceReference(m_gloveRightBuzzMotorReference);
            }
            /**else if (m_timeNow - m_timePreparationStarting > 5.0
                       && m_timeNow - m_timePreparationStarting < 10.0)
            {
                m_gloveRightHand->setPalmFeedbackThumper(1);
            }
            else if (m_timeNow - m_timePreparationStarting > 10.0
                       && m_timeNow - m_timePreparationStarting < 15.0)
            {
                m_gloveRightHand->setPalmFeedbackThumper(2);
            } */
            else
            {
                yInfo() << "----------------------------- time: "
                        << m_timeNow - m_timePreparation << " stop command";

                m_gloveRightHand->stopFeedback();
                m_timePreparation = m_timeNow;
            }
        }
        // if (((int)tmp_buzzVal)%4==0)
        //    m_gloveRightHand->setPalmFeedback(0);

        yInfo() << "**************";
        // yInfo() << "gloveSensors: \n" << gloveSensors;
        // yInfo() << "**************";
        // std::cout << "hand pose: \n" << handPose << std::endl;
        // yInfo() << "**************";
        // std::cout << "glove pose: \n" << glovePose << std::endl;
        // yInfo() << "handJointsAngles.size :" << rightHandJointsAngles.rows() <<
        // rightHandJointsAngles.cols(); yInfo() << "index finger, first joint: " <<
        // rightHandJointsAngles(4, 0)
        //        << rightHandJointsAngles(4, 1) << rightHandJointsAngles(4, 2);

        // yInfo() << "index finger, second joint: " << rightHandJointsAngles(5, 0)
        //        << rightHandJointsAngles(5, 1) << rightHandJointsAngles(5, 2);

        // yInfo() << "index finger, third joint: " << rightHandJointsAngles(6, 0)
        //        << rightHandJointsAngles(6, 1) << rightHandJointsAngles(6, 2);

        // yInfo() << "index finger, fourth joint: " << rightHandJointsAngles(7, 0)
        //        << rightHandJointsAngles(7, 1) << rightHandJointsAngles(7, 2);

        // std::cout << "handJointsAngles: \n" << rightHandJointsAngles << std::endl;

        // yInfo() << "handJointsAngles.size :" << leftHandJointsAngles.rows() <<
        //        leftHandJointsAngles.cols();
        // yInfo() << "index finger, first joint: " << leftHandJointsAngles(4, 0)
        //         << leftHandJointsAngles(4, 1) << leftHandJointsAngles(4, 2);

        // yInfo() << "index finger, second joint: " << leftHandJointsAngles(5, 0)
        //        << leftHandJointsAngles(5, 1) << leftHandJointsAngles(5, 2);

        // yInfo() << "index finger, third joint: " << leftHandJointsAngles(6, 0)
        //         << leftHandJointsAngles(6, 1) << leftHandJointsAngles(6, 2);

        // yInfo() << "index finger, fourth joint: " << leftHandJointsAngles(7, 0)
        //         << leftHandJointsAngles(7, 1) << leftHandJointsAngles(7, 2);

        // 2- Compute the reference values for the haptic glove, including resistance force and
        // vibrotactile feedback

        // 3- Set the reference joint valued for the iCub hand fingers
        // left hand
        //        m_leftHandFingers->setFingersAxisReference(m_icubLeftFingerAxisReference);
/*
        //TODEL--> START
        if (m_useLeftHand)
        {
            std::cerr << "Debug: 07 \n";
            Eigen::MatrixXd leftHandPose, leftGlovePose, leftHandJointsAngles;
            std::vector<float> leftGloveSensors;

            m_gloveLeftHand->getHandPose(leftHandPose);
            m_gloveLeftHand->getGlovePose(leftGlovePose);
            m_gloveLeftHand->getSensorData(leftGloveSensors);
            m_gloveLeftHand->getHandJointsAngles(leftHandJointsAngles);

            m_icubLeftFingerJointsReference.zero();

            int i = 0;
            // thumb: (i:i+2)
            m_icubLeftFingerJointsReference(i + 0) = -leftHandJointsAngles(0, 2) + 0.7;
            m_icubLeftFingerJointsReference(i + 1) = leftHandJointsAngles(1, 1);
            m_icubLeftFingerJointsReference(i + 2) = leftHandJointsAngles(2, 1);

            // index (i+3:i+5)
            m_icubLeftFingerJointsReference(i + 3) = leftHandJointsAngles(4, 1);
            m_icubLeftFingerJointsReference(i + 4) = leftHandJointsAngles(5, 1);
            m_icubLeftFingerJointsReference(i + 5) = leftHandJointsAngles(6, 1);

            // middle (i+6:i+8)
            m_icubLeftFingerJointsReference(i + 6) = leftHandJointsAngles(8, 1);
            m_icubLeftFingerJointsReference(i + 7) = leftHandJointsAngles(9, 1);
            m_icubLeftFingerJointsReference(i + 8) = leftHandJointsAngles(10, 1);

            // ring (i+9:i+11)
            m_icubLeftFingerJointsReference(i + 9) = leftHandJointsAngles(12, 1);
            m_icubLeftFingerJointsReference(i + 10) = leftHandJointsAngles(13, 1);
            m_icubLeftFingerJointsReference(i + 11) = leftHandJointsAngles(14, 1);

            // pinkie (i+12:i+14)
            m_icubLeftFingerJointsReference(i + 12) = leftHandJointsAngles(16, 1);
            m_icubLeftFingerJointsReference(i + 13) = leftHandJointsAngles(17, 1);
            m_icubLeftFingerJointsReference(i + 14) = leftHandJointsAngles(18, 1);
            yInfo() << "m_icubLeftFingerJointsReference: "
                    << m_icubLeftFingerJointsReference.toString();
        }

        if (m_useRightHand)
        {
            std::cerr << "Debug: 08 \n";
            Eigen::MatrixXd rightHandPose, rightGlovePose, rightHandJointsAngles;
            std::vector<float> rightGloveSensors;

            m_gloveRightHand->getHandPose(rightHandPose);
            m_gloveRightHand->getGlovePose(rightGlovePose);
            m_gloveRightHand->getSensorData(rightGloveSensors);
            m_gloveRightHand->getHandJointsAngles(rightHandJointsAngles);

            m_icubRightFingerJointsReference.zero();

            int i = 0;
            // iff only index
            // index (i+3:i+5)
            //m_icubRightFingerJointsReference(i + 0) = rightHandJointsAngles(4, 1);
            //m_icubRightFingerJointsReference(i + 1) = rightHandJointsAngles(5, 1);
            //m_icubRightFingerJointsReference(i + 2) = rightHandJointsAngles(6, 1);

            // thumb + index
            //m_icubRightFingerJointsReference(i + 0) = rightHandJointsAngles(1, 1);
            //m_icubRightFingerJointsReference(i + 1) = rightHandJointsAngles(2, 1);
            //m_icubRightFingerJointsReference(i + 2) = rightHandJointsAngles(4, 1);
            //m_icubRightFingerJointsReference(i + 3) = rightHandJointsAngles(5, 1);
            //m_icubRightFingerJointsReference(i + 4) = rightHandJointsAngles(6, 1);

            // thumb: (i:i+2)
            m_icubRightFingerJointsReference(i + 0) = rightHandJointsAngles(0, 2)+0.7;
            m_icubRightFingerJointsReference(i + 1) = rightHandJointsAngles(1, 1);
            m_icubRightFingerJointsReference(i + 2) = rightHandJointsAngles(2, 1);

            // index (i+3:i+5)
            m_icubRightFingerJointsReference(i + 3) = rightHandJointsAngles(4, 1);
            m_icubRightFingerJointsReference(i + 4) = rightHandJointsAngles(5, 1);
            m_icubRightFingerJointsReference(i + 5) = rightHandJointsAngles(6, 1);

            // middle (i+6:i+8)
            m_icubRightFingerJointsReference(i + 6) = rightHandJointsAngles(8, 1);
            m_icubRightFingerJointsReference(i + 7) = rightHandJointsAngles(9, 1);
            m_icubRightFingerJointsReference(i + 8) = rightHandJointsAngles(10, 1);

            // ring (i+9:i+11)
            m_icubRightFingerJointsReference(i + 9) = rightHandJointsAngles(12, 1);
            m_icubRightFingerJointsReference(i + 10) = rightHandJointsAngles(13, 1);
            m_icubRightFingerJointsReference(i + 11) = rightHandJointsAngles(14, 1);

//            // pinkie (i+12:i+14)
            m_icubRightFingerJointsReference(i + 12) = rightHandJointsAngles(16, 1);
            m_icubRightFingerJointsReference(i + 13) = rightHandJointsAngles(17, 1);
            m_icubRightFingerJointsReference(i + 14) = rightHandJointsAngles(18, 1);
            yInfo() << "m_icubRightFingerJointsReference: "
                    << m_icubRightFingerJointsReference.toString();
        }
        //TODEL--> END
*/
        if (m_useLeftHand)
        {
            std::vector<double> humanJointAngles;
            m_gloveLeftHand->getHandJointsAngles(humanJointAngles);
//            yInfo()<<"human joint angles: "<<humanJointAngles;
            if(!m_retargetingLeftHand->retargetHumanMotionToRobot(humanJointAngles))
            {
                yError()<<"[HapticGloveModule::updateModule()] m_retargetingLeftHand->retargetHumanMotionToRobot returns false! ";
                return false;
            }
            m_retargetingLeftHand->getRobotJointReferences(m_icubLeftFingerJointsReference);
            yInfo()<<"m_icubLeftFingerJointsReference: "<<m_icubLeftFingerJointsReference.toString();
        }
        if (m_useRightHand)
        {
            std::vector<double> humanJointAngles;
            m_gloveRightHand->getHandJointsAngles(humanJointAngles);
            if(!m_retargetingRightHand->retargetHumanMotionToRobot(humanJointAngles))
            {
                yError()<<"[HapticGloveModule::updateModule()] m_retargetingRightHand->retargetHumanMotionToRobot returns false! ";
                return false;
            }
            m_retargetingRightHand->getRobotJointReferences(m_icubRightFingerJointsReference);
        }
        double time2=yarp::os::Time::now();
        yInfo()<<"[updateModule] time retargeting h->r: "<<time2-time1;




        /*** COMPUTE FORCE FEEDBACK***/
        if (m_useLeftHand)
        {

            std::vector<double> axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF;
            std::vector<double> axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF;
            Eigen::MatrixXd feedbackAxisCovEstimationKF, referenceAxisCovEstimationKF;

            m_robotLeftHand->getEstimatedMotorsState(axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF, feedbackAxisCovEstimationKF,
                                                     axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF, referenceAxisCovEstimationKF);

            for (int i=0; i<m_icubLeftFingerAxisValueReference.size();i++)
            {
                m_icubLeftFingerAxisValueError(i)= axisReferenceValuesEstimationKF[i] - axisFeedbackValuesEstimationKF[i];
                m_icubLeftFingerAxisVelocityError(i)= axisReferenceVelocitiesEstimationKF[i] - axisFeedbackVelocitiesEstimationKF[i];

                m_icubLeftFingerAxisValueErrorSmoothed(i)= m_icubLeftFingerAxisValueError(i);
                m_icubLeftFingerAxisVelocityErrorSmoothed(i)=m_icubLeftFingerAxisVelocityError(i);
            }

            for (int i=0; i<m_icubLeftFingerAxisValueReference.size();i++)
            {
                // the robot cannot mechnically go below zero, so if this is the case then we set the error to zero
                if(axisReferenceValuesEstimationKF[i] <0)
                {
                    m_icubLeftFingerAxisValueErrorSmoothed(i)=0.0;
                    m_icubLeftFingerAxisVelocityErrorSmoothed(i)= 0.0;
                }
                bool isInContact=true;
                isInContact=
//                        (std::abs(axisFeedbackVelocitiesEstimationKF[i]) < m_velocity_threshold_transient) &&
                        (std::abs(m_icubLeftFingerAxisValueErrorSmoothed[i]) > m_Value_error_threshold_transient)
                        ;

                if(!isInContact)
                {
                    m_icubLeftFingerAxisValueErrorSmoothed(i)=0.0;
                    m_icubLeftFingerAxisVelocityErrorSmoothed(i)=0.0;
                }
            }
            yInfo()<<"axisReferenceValuesEstimationKF: "<<axisReferenceValuesEstimationKF;
            yInfo()<<"axisFeedbackValuesEstimationKF: "<<axisFeedbackValuesEstimationKF;
            yInfo()<<"axisReferenceVelocitiesEstimationKF: "<<axisReferenceVelocitiesEstimationKF;
            yInfo()<<"axisFeedbackVelocitiesEstimationKF: "<<axisFeedbackVelocitiesEstimationKF;
            yInfo()<<"m_icubLeftFingerAxisValueErrorSmoothed: "<<m_icubLeftFingerAxisValueErrorSmoothed.toString();
            yInfo()<<"m_icubLeftFingerAxisVelocityErrorSmoothed: "<<m_icubLeftFingerAxisVelocityErrorSmoothed.toString();


            // Deleted to use KALMAN FILTER
/*            for (int i=0; i<m_icubLeftFingerAxisValueReference.size();i++)
            {
                m_icubLeftFingerAxisValueError(i)= m_icubLeftFingerAxisValueReference(i) - m_icubLeftFingerAxisValueFeedback(i);
                m_icubLeftFingerAxisVelocityError(i)= m_icubLeftFingerAxisVelocityReference(i) - m_icubLeftFingerAxisVelocityFeedback(i);
            }

            m_leftAxisValueErrorSmoother->computeNextValues(m_icubLeftFingerAxisValueError);
            m_icubLeftFingerAxisValueErrorSmoothed = m_leftAxisValueErrorSmoother->getPos();

            m_leftAxisVelocityErrorSmoother->computeNextValues(m_icubLeftFingerAxisVelocityError);
            m_icubLeftFingerAxisVelocityErrorSmoothed = m_leftAxisVelocityErrorSmoother->getPos();


            // FILTERS
            double velocity_threshold_transient= 0.6;

            for (int i=0; i<m_icubLeftFingerAxisValueReference.size();i++)
            {
                // the robot cannot mechnically go below zero, so if this is the case then we set the error to zero
                if(m_icubLeftFingerAxisValueReference(i) <0)
                {
                    m_icubLeftFingerAxisValueErrorSmoothed(i)=0.0;
                    m_icubLeftFingerAxisVelocityErrorSmoothed(i)= 0.0;
                }

                if(std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(i)) > velocity_threshold_transient)
                {
                    m_icubLeftFingerAxisVelocityErrorSmoothed(i)=0.0;
                    m_icubLeftFingerAxisVelocityErrorSmoothed(i)=0.0;
                }
            }
            */

//            if(std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(1)) > velocity_threshold_transient ||
//                    std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(2)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this thumb
//                m_icubLeftFingerAxisVelocityErrorSmoothed(1)=0.0;
//                m_icubLeftFingerAxisVelocityErrorSmoothed(2)=0.0;

//                m_icubLeftFingerAxisValueErrorSmoothed(1)=0.0;
//                m_icubLeftFingerAxisValueErrorSmoothed(2)=0.0;
//            }

//            if(std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(3)) > velocity_threshold_transient ||
//                    std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(4)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this index
//                m_icubLeftFingerAxisVelocityErrorSmoothed(3)=0.0;
//                m_icubLeftFingerAxisVelocityErrorSmoothed(4)=0.0;

//                m_icubLeftFingerAxisValueErrorSmoothed(3)=0.0;
//                m_icubLeftFingerAxisValueErrorSmoothed(4)=0.0;
//            }

//            if(std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(5)) > velocity_threshold_transient ||
//                    std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(6)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this ring
//                m_icubLeftFingerAxisVelocityErrorSmoothed(5)=0.0;
//                m_icubLeftFingerAxisVelocityErrorSmoothed(6)=0.0;

//                m_icubLeftFingerAxisValueErrorSmoothed(5)=0.0;
//                m_icubLeftFingerAxisValueErrorSmoothed(6)=0.0;
//            }

//            if(std::abs(m_icubLeftFingerAxisVelocityErrorSmoothed(7)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this pinky
//                m_icubLeftFingerAxisVelocityErrorSmoothed(7)=0.0;
//                m_icubLeftFingerAxisValueErrorSmoothed(7)=0.0;
//            }


            // "r_thumb_oppose":0 , "r_thumb_proximal":1, "r_thumb_distal":2, "r_index_proximal":3,
            // "r_index-distal":4, "r_middle-proximal":5, "r_middle-distal":6, "r_little-fingers":7
/*
            //TODEL-->START
            yInfo()<<"Left Axis thumb: "<<m_icubLeftFingerAxisValueErrorSmoothed(2)<<" , "
                  << m_icubLeftFingerAxisValueErrorSmoothed(1)<<" , " <<
                     m_icubLeftFingerAxisValueErrorSmoothed(0);
            yInfo()<<"Left Axis index: "<<m_icubLeftFingerAxisValueErrorSmoothed(4)<< " , "
                  << m_icubLeftFingerAxisValueErrorSmoothed(3);
            yInfo()<<"Left Axis middle: "<<m_icubLeftFingerAxisValueErrorSmoothed(6)<< " , "
                  << m_icubLeftFingerAxisValueErrorSmoothed(5);
            yInfo()<<"Left Axis springy: "<<m_icubLeftFingerAxisValueErrorSmoothed(7);

            //            for (int i = 0; i < m_gloveRightForceFeedbackReference.size(); i++)



            m_retargetingLeftHand->retargetForceFeedbackFromRobotToHuman(m_icubLeftFingerAxisValueErrorSmoothed, m_icubLeftFingerAxisVelocityErrorSmoothed);
            m_gloveLeftForceFeedbackReference(0)=
                    m_leftTotalGain(0)* (m_icubLeftFingerAxisValueErrorSmoothed(0) + m_leftVelocityGain(0)*m_icubLeftFingerAxisVelocityErrorSmoothed(0)) +
                    m_leftTotalGain(1)* (m_icubLeftFingerAxisValueErrorSmoothed(1) + m_leftVelocityGain(1)*m_icubLeftFingerAxisVelocityErrorSmoothed(1)) +
                    m_leftTotalGain(2)* (m_icubLeftFingerAxisValueErrorSmoothed(2) + m_leftVelocityGain(2)*m_icubLeftFingerAxisVelocityErrorSmoothed(2)) ;

            m_gloveLeftForceFeedbackReference(1)=
                    m_leftTotalGain(3)* (m_icubLeftFingerAxisValueErrorSmoothed(3) + m_leftVelocityGain(3)*m_icubLeftFingerAxisVelocityErrorSmoothed(3)) +
                    m_leftTotalGain(4)* (m_icubLeftFingerAxisValueErrorSmoothed(4) + m_leftVelocityGain(4)*m_icubLeftFingerAxisVelocityErrorSmoothed(4)) ;

            m_gloveLeftForceFeedbackReference(2)=
                    m_leftTotalGain(5)* (m_icubLeftFingerAxisValueErrorSmoothed(5) + m_leftVelocityGain(5)*m_icubLeftFingerAxisVelocityErrorSmoothed(5)) +
                    m_leftTotalGain(6)* (m_icubLeftFingerAxisValueErrorSmoothed(6) + m_leftVelocityGain(6)*m_icubLeftFingerAxisVelocityErrorSmoothed(6)) ;


            m_gloveLeftForceFeedbackReference(3)=
                    m_leftTotalGain(7)* (m_icubLeftFingerAxisValueErrorSmoothed(7) + m_leftVelocityGain(7)*m_icubLeftFingerAxisVelocityErrorSmoothed(7));

            m_gloveLeftForceFeedbackReference(4)=
                    m_leftTotalGain(7)* (m_icubLeftFingerAxisValueErrorSmoothed(7) + m_leftVelocityGain(7)*m_icubLeftFingerAxisVelocityErrorSmoothed(7));

//                    (m_icubLeftFingerAxisValueErrorSmoothed(2)
//                                                      + m_icubLeftFingerAxisValueErrorSmoothed(1));
            //                    + std::abs(icubRightFingerAxisFeedback[0]-icubRightFingerAxisReference[0])
//            m_gloveLeftForceFeedbackReference(1)=150*(m_icubLeftFingerAxisValueErrorSmoothed(4)
//                                                      + m_icubLeftFingerAxisValueErrorSmoothed(3));
//            m_gloveLeftForceFeedbackReference(2)=50*(m_icubLeftFingerAxisValueErrorSmoothed(6)
//                                                     + m_icubLeftFingerAxisValueErrorSmoothed(5));
//            m_gloveLeftForceFeedbackReference(3)=100*(m_icubLeftFingerAxisValueErrorSmoothed(7));
//            m_gloveLeftForceFeedbackReference(4)=100*std::abs(m_icubLeftFingerAxisValueErrorSmoothed(7));

            yInfo()<<"Hand Feedback Finger 0: "<<m_gloveLeftForceFeedbackReference(0);
            yInfo()<<"Hand Feedback Finger 1: "<<m_gloveLeftForceFeedbackReference(1);
            yInfo()<<"Hand Feedback Finger 2: "<<m_gloveLeftForceFeedbackReference(2);
            yInfo()<<"Hand Feedback Finger 3: "<<m_gloveLeftForceFeedbackReference(3);
            yInfo()<<"Hand Feedback Finger 4: "<<m_gloveLeftForceFeedbackReference(4);

            for (int i = 0; i < 5; i++)
            {
                m_gloveLeftBuzzMotorReference(i) = m_leftBuzzMotorsGain(i)* m_gloveLeftForceFeedbackReference(i);
            }
            //TODEL-->END
*/

            m_retargetingLeftHand->retargetHapticFeedbackFromRobotToHuman(m_icubLeftFingerAxisValueErrorSmoothed, m_icubLeftFingerAxisVelocityErrorSmoothed);
            m_retargetingLeftHand->getForceFeedbackToHuman(m_gloveLeftForceFeedbackReference);
            m_retargetingLeftHand->getVibroTactileFeedbackToHuman(m_gloveLeftBuzzMotorReference);

        }
        if (m_useRightHand)
        {
            std::vector<double> axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF;
            std::vector<double> axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF;
            Eigen::MatrixXd feedbackAxisCovEstimationKF, referenceAxisCovEstimationKF;

            m_robotRightHand->getEstimatedMotorsState(axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF, feedbackAxisCovEstimationKF,
                                                     axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF, referenceAxisCovEstimationKF);

            for (int i=0; i<m_icubRightFingerAxisValueReference.size();i++)
            {
                m_icubRightFingerAxisValueError(i)= axisReferenceValuesEstimationKF[i] - axisFeedbackValuesEstimationKF[i];
                m_icubRightFingerAxisVelocityError(i)= axisReferenceVelocitiesEstimationKF[i] - axisFeedbackVelocitiesEstimationKF[i];

                m_icubRightFingerAxisValueErrorSmoothed(i)    = m_icubRightFingerAxisValueError(i);
                m_icubRightFingerAxisVelocityErrorSmoothed(i) = m_icubRightFingerAxisVelocityError(i);
            }


            //            std::vector<double> icubRightFingerAxisFeedback, icubRightFingerAxisReference;

            for (int i=0; i<m_icubRightFingerAxisValueReference.size();i++)
            {
                m_icubRightFingerAxisValueError(i)=  m_icubRightFingerAxisValueReference(i)- m_icubRightFingerAxisValueFeedback(i);
                m_icubRightFingerAxisVelocityError(i)=  m_icubRightFingerAxisVelocityReference(i)- m_icubRightFingerAxisVelocityFeedback(i);

            }
            m_rightAxisValueErrorSmoother->computeNextValues(m_icubRightFingerAxisValueError);
            m_icubRightFingerAxisValueErrorSmoothed = m_rightAxisValueErrorSmoother->getPos();

            m_rightAxisVelocityErrorSmoother->computeNextValues(m_icubRightFingerAxisVelocityError);
            m_icubRightFingerAxisVelocityErrorSmoothed = m_rightAxisVelocityErrorSmoother->getPos();

            // FILTERS
            double velocity_threshold_transient= 0.6;

            for (int i=0; i<m_icubRightFingerAxisValueReference.size();i++)
            {
                // the robot cannot mechnically go below zero, so if this is the case then we set the error to zero
                if(m_icubRightFingerAxisValueReference(i) <0)
                {
                    m_icubRightFingerAxisValueErrorSmoothed(i)=0.0;
                    m_icubRightFingerAxisVelocityErrorSmoothed(i)= 0.0;
                }

                bool isInContact=true;
                isInContact=
//                        (std::abs(axisFeedbackVelocitiesEstimationKF[i]) < m_velocity_threshold_transient) &&
                        (std::abs(m_icubRightFingerAxisValueErrorSmoothed[i]) > m_Value_error_threshold_transient)
                        ;

                if(!isInContact)
                {
                    m_icubRightFingerAxisValueErrorSmoothed(i)=0.0;
                    m_icubRightFingerAxisVelocityErrorSmoothed(i)=0.0;
                }


//                if(std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(i)) > velocity_threshold_transient)
//                {
//                    // we are in transient phase for this thumb
//                    m_icubRightFingerAxisVelocityErrorSmoothed(i)=0.0;
//                    m_icubRightFingerAxisVelocityErrorSmoothed(i)=0.0;
//                }

            }
//            if(std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(1)) > velocity_threshold_transient ||
//                    std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(2)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this thumb
//                m_icubRightFingerAxisVelocityErrorSmoothed(1)=0.0;
//                m_icubRightFingerAxisVelocityErrorSmoothed(2)=0.0;

//                m_icubRightFingerAxisValueErrorSmoothed(1)=0.0;
//                m_icubRightFingerAxisValueErrorSmoothed(2)=0.0;
//            }

//            if(std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(3)) > velocity_threshold_transient ||
//                    std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(4)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this index
//                m_icubRightFingerAxisVelocityErrorSmoothed(3)=0.0;
//                m_icubRightFingerAxisVelocityErrorSmoothed(4)=0.0;

//                m_icubRightFingerAxisValueErrorSmoothed(3)=0.0;
//                m_icubRightFingerAxisValueErrorSmoothed(4)=0.0;
//            }

//            if(std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(5)) > velocity_threshold_transient ||
//                    std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(6)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this ring
//                m_icubRightFingerAxisVelocityErrorSmoothed(5)=0.0;
//                m_icubRightFingerAxisVelocityErrorSmoothed(6)=0.0;

//                m_icubRightFingerAxisValueErrorSmoothed(5)=0.0;
//                m_icubRightFingerAxisValueErrorSmoothed(6)=0.0;
//            }

//            if(std::abs(m_icubRightFingerAxisVelocityErrorSmoothed(7)) > velocity_threshold_transient)
//            {
//                // we are in transient phase for this pinky
//                m_icubRightFingerAxisVelocityErrorSmoothed(7)=0.0;
//                m_icubRightFingerAxisValueErrorSmoothed(7)=0.0;
//            }




            // "r_thumb_oppose":0 , "r_thumb_proximal":1, "r_thumb_distal":2, "r_index_proximal":3,
            // "r_index-distal":4, "r_middle-proximal":5, "r_middle-distal":6, "r_little-fingers":7

//            yInfo()<<"Right Axis thumb: "<<m_icubRightFingerAxisValueErrorSmoothed(2)<<" , "
//                  << m_icubRightFingerAxisValueErrorSmoothed(1)<<" , " <<
//                     m_icubRightFingerAxisValueErrorSmoothed(0);
//            yInfo()<<"Right Axis index: "<<m_icubRightFingerAxisValueErrorSmoothed(4)<< " , "
//                  << m_icubRightFingerAxisValueErrorSmoothed(3);
//            yInfo()<<"Right Axis middle: "<<m_icubRightFingerAxisValueErrorSmoothed(6)<< " , "
//                  << m_icubRightFingerAxisValueErrorSmoothed(5);
//            yInfo()<<"Right Axis springy: "<<m_icubRightFingerAxisValueErrorSmoothed(7);
            int k_gain=150;
            //            for (int i = 0; i < m_gloveRightForceFeedbackReference.size(); i++)


/*
            //TODEL-->START
            m_gloveRightForceFeedbackReference(0)=
                    m_rightTotalGain(0)* (m_icubRightFingerAxisValueErrorSmoothed(0) + m_rightVelocityGain(0)*m_icubRightFingerAxisVelocityErrorSmoothed(0)) +
                    m_rightTotalGain(1)* (m_icubRightFingerAxisValueErrorSmoothed(1) + m_rightVelocityGain(1)*m_icubRightFingerAxisVelocityErrorSmoothed(1)) +
                    m_rightTotalGain(2)* (m_icubRightFingerAxisValueErrorSmoothed(2) + m_rightVelocityGain(2)*m_icubRightFingerAxisVelocityErrorSmoothed(2)) ;

            m_gloveRightForceFeedbackReference(1)=
                    m_rightTotalGain(3)* (m_icubRightFingerAxisValueErrorSmoothed(3) + m_rightVelocityGain(3)*m_icubRightFingerAxisVelocityErrorSmoothed(3)) +
                    m_rightTotalGain(4)* (m_icubRightFingerAxisValueErrorSmoothed(4) + m_rightVelocityGain(4)*m_icubRightFingerAxisVelocityErrorSmoothed(4)) ;

            m_gloveRightForceFeedbackReference(2)=
                    m_rightTotalGain(5)* (m_icubRightFingerAxisValueErrorSmoothed(5) + m_rightVelocityGain(5)*m_icubRightFingerAxisVelocityErrorSmoothed(5)) +
                    m_rightTotalGain(6)* (m_icubRightFingerAxisValueErrorSmoothed(6) + m_rightVelocityGain(6)*m_icubRightFingerAxisVelocityErrorSmoothed(6)) ;


            m_gloveRightForceFeedbackReference(3)=
                    m_rightTotalGain(7)* (m_icubRightFingerAxisValueErrorSmoothed(7) + m_rightVelocityGain(7)*m_icubRightFingerAxisVelocityErrorSmoothed(7));

            m_gloveRightForceFeedbackReference(4)=
                    m_rightTotalGain(7)* (m_icubRightFingerAxisValueErrorSmoothed(7) + m_rightVelocityGain(7)*m_icubRightFingerAxisVelocityErrorSmoothed(7));

std::cerr<<"106 \n";

//            m_gloveRightForceFeedbackReference(0)=150*(m_icubRightFingerAxisValueErrorSmoothed(2)
//                                                       + m_icubRightFingerAxisValueErrorSmoothed(1));
//            //                    + std::abs(icubRightFingerAxisFeedback[0]-icubRightFingerAxisReference[0])
//            m_gloveRightForceFeedbackReference(1)=150*(m_icubRightFingerAxisValueErrorSmoothed(4)
//                                                       + m_icubRightFingerAxisValueErrorSmoothed(3));
//            m_gloveRightForceFeedbackReference(2)=50*(m_icubRightFingerAxisValueErrorSmoothed(6)
//                                                      + m_icubRightFingerAxisValueErrorSmoothed(5));
//            m_gloveRightForceFeedbackReference(3)=100*(m_icubRightFingerAxisValueErrorSmoothed(7));
//            m_gloveRightForceFeedbackReference(4)=100*std::abs(m_icubRightFingerAxisValueErrorSmoothed(7));

            yInfo()<<"Hand Feedback Finger 0: "<<m_gloveRightForceFeedbackReference(0);
            yInfo()<<"Hand Feedback Finger 1: "<<m_gloveRightForceFeedbackReference(1);
            yInfo()<<"Hand Feedback Finger 2: "<<m_gloveRightForceFeedbackReference(2);
            yInfo()<<"Hand Feedback Finger 3: "<<m_gloveRightForceFeedbackReference(3);
            yInfo()<<"Hand Feedback Finger 4: "<<m_gloveRightForceFeedbackReference(4);

            for (int i = 0; i < 5; i++)
            {m_gloveRightBuzzMotorReference(i) = m_rightBuzzMotorsGain(i)* m_gloveRightForceFeedbackReference(i);}

*/
            //TODEL-->END

            m_retargetingRightHand->retargetHapticFeedbackFromRobotToHuman(m_icubRightFingerAxisValueErrorSmoothed, m_icubRightFingerAxisVelocityErrorSmoothed);
            m_retargetingRightHand->getForceFeedbackToHuman(m_gloveRightForceFeedbackReference);
            m_retargetingRightHand->getVibroTactileFeedbackToHuman(m_gloveRightBuzzMotorReference);



        }
        double time3=yarp::os::Time::now();
        yInfo()<<"[updateModule] time retargeting r->h: "<<time3-time2;

        if (m_useLeftHand)
        {
            // set robot values
            yInfo()<<"m_icubLeftFingerJointsReference\n"<<m_icubLeftFingerJointsReference.toString();

            m_robotLeftHand->setFingersJointReference(m_icubLeftFingerJointsReference);
            m_robotLeftHand->move();
            // set glove values
            yInfo()<<"m_gloveLeftForceFeedbackReference\n"<<m_gloveLeftForceFeedbackReference.toString();
            yInfo()<<"m_gloveLeftBuzzMotorReference\n"<<m_gloveLeftBuzzMotorReference.toString();
            m_gloveLeftHand->setFingersForceReference(m_gloveLeftForceFeedbackReference);
            m_gloveLeftHand->setBuzzMotorsReference(m_gloveLeftBuzzMotorReference);
        }

        if (m_useRightHand)
        {
            // set robot values
            yInfo()<<"m_icubRightFingerJointsReference\n"<<m_icubRightFingerJointsReference.toString();
            m_robotRightHand->setFingersJointReference(m_icubRightFingerJointsReference);
            m_robotRightHand->move();

            // set glove values
            yInfo()<<"m_gloveRightForceFeedbackReference\n"<<m_gloveRightForceFeedbackReference.toString();
            yInfo()<<"m_gloveRightBuzzMotorReference\n"<<m_gloveRightBuzzMotorReference.toString();

            m_gloveRightHand->setFingersForceReference(m_gloveRightForceFeedbackReference);
            m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);
        }
        double time4=yarp::os::Time::now();
        yInfo()<<"[updateModule] time setting values: "<<time4-time3;

        // right hand
        //        m_rightHandFingers->setFingersAxisReference(m_icubRightFingerAxisReference);
        //        m_rightHandFingers->move();

        // 4- Set the reference values for the haptic glove, including resistance force and
        // vibrotactile feedback
        logData();
        double time5= yarp::os::Time::now();
        yWarning()<<"[updateModule] time logger:"<< time5-time4;

        // yInfo() << "[HapticGloveModule::updateModule] <FSM::InPreparation> exiting ....";
        // exit(0);
        yWarning()<<"[updateModule] time total update:"<< time5-time0;


    } else if (m_state == HapticGloveFSM::Configured)
    {
        yInfo() << " >>> STATUS: Configured";

        // TODO

        if (m_useLeftHand)
        {
            if (!m_gloveLeftHand->setupGlove())
            {
                yError() << "[HapticGloveModule::updateModule()] cannot setup the left hand glove.";
                return false;
            }
        }

        if (m_useRightHand)
        {
            if (!m_gloveRightHand->setupGlove())
            {
                yError()
                        << "[HapticGloveModule::updateModule()] cannot setup the right hand glove.";
                return false;
            }
        }

        m_timeConfigurationEnding = yarp::os::Time::now();
        m_state = HapticGloveFSM::InPreparation;
        yInfo()<<"the state changed from HapticGloveFSM::Configured to HapticGloveFSM::InPreparation";
    } else if (m_state == HapticGloveFSM::InPreparation)
    {
        yInfo() << " >>> STATUS: InPreparation";

        m_timePreparation = yarp::os::Time::now();

        // TODO
        //        if (m_timePreparationStarting - m_timeConfigurationStarting > 10.0)
        //        {

        //            m_state = HapticGloveFSM::Running;
        //            yInfo() << "[HapticGloveModule::updateModule] start the haptic glove module";
        //            yInfo() << "[HapticGloveModule::updateModule] Running ...";
        //        } else
        //        {

        //            yInfo() << "time collecting data:"
        //                    << m_timePreparationStarting - m_timeConfigurationStarting;
        //            int dTime = int((m_timePreparationStarting - m_timeConfigurationStarting) /
        //            m_dT); if (dTime % 20 == 0 || std::abs(dTime - 1) < 0.1)
        //                m_leftHandFingers->LogDataToCalibrateRobotMotorsJointsCouplingRandom(true);
        //            else
        //            {
        //                m_leftHandFingers->LogDataToCalibrateRobotMotorsJointsCouplingRandom(false);
        //            }
        //        }
        yInfo() << "time collecting data:"
                << m_timePreparation - m_timeConfigurationEnding;
        int dTime = int((m_timePreparation - m_timeConfigurationEnding) / m_dT);
        int CouplingConstant=(int)(m_calibrationTimePeriod/m_dT);


        int axisNumber = int(dTime / CouplingConstant); // both operands needs to be integer
//        axisNumber = dTime / 500;

        if (m_useLeftHand)
        {
            if (axisNumber >= m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis())
            {
                yInfo() << "******>>>> Data collected ...";
                if(!m_robotLeftHand->isRobotPrepared())
                {
                    if (!m_robotLeftHand->trainCouplingMatrix() )
                    {
                        yInfo()
                                << "cannot claibrate the coupling matrix and find the coefficient matrix";
                        return false;
                    }
                }
                if(m_getHumanMotionRange)
                {
                    std::vector<double> humanHandJointRangeMin, humanHandJointRangeMax;
                    m_gloveLeftHand->getHumanMotionRange(humanHandJointRangeMin,humanHandJointRangeMax);
                    m_retargetingLeftHand->computeJointAngleRetargetingParams(humanHandJointRangeMin,humanHandJointRangeMax);
                }

//                m_timePreparationStarting = yarp::os::Time::now();
//                m_state = HapticGloveFSM::Running;

            } else
            {
                double time = double(dTime % CouplingConstant)* m_dT
                        * (M_PI / m_calibrationTimePeriod);
//                time = double(dTime % 500) / 500.0 * (M_PI * 2.0);

                m_robotLeftHand->LogDataToCalibrateRobotMotorsJointsCouplingSin(time, axisNumber);
                if(m_getHumanMotionRange)
                {
                    m_gloveLeftHand->findHumanMotionRange();
                }
            }

        }

        if (m_useRightHand)
        {
            if (axisNumber >= m_robotRightHand->controlHelper()->getNumberOfActuatedAxis() )
            {
                yInfo() << "******>>>> Data collected ...";
                if(!m_robotRightHand->isRobotPrepared())
                {
                    if (!m_robotRightHand->trainCouplingMatrix())
                    {
                        yInfo()
                                << "cannot claibrate the coupling matrix and find the coefficient matrix";
                        return false;
                    }
                    if(m_getHumanMotionRange)
                    {
                        std::vector<double> humanHandJointRangeMin, humanHandJointRangeMax;
                        m_gloveRightHand->getHumanMotionRange(humanHandJointRangeMin,humanHandJointRangeMax);
                        m_retargetingRightHand->computeJointAngleRetargetingParams(humanHandJointRangeMin,humanHandJointRangeMax);
                    }
                }
//                m_timePreparationStarting = yarp::os::Time::now();
//                m_state = HapticGloveFSM::Running;


            } else
            {
                double time = double(dTime % CouplingConstant)* m_dT
                        * (M_PI / m_calibrationTimePeriod);
//                double time = double(dTime % 500) / 500.0 * (M_PI * 2.0);
                m_robotRightHand->LogDataToCalibrateRobotMotorsJointsCouplingSin(time,
                                                                                 axisNumber);
                if(m_getHumanMotionRange)
                {
                    m_gloveRightHand->findHumanMotionRange();
                }
            }

        }

        if(m_useLeftHand && m_useRightHand)
        {
            if(m_robotLeftHand->isRobotPrepared() && m_robotRightHand->isRobotPrepared())
            {
                m_state = HapticGloveFSM::Running;
                m_timePreparation = yarp::os::Time::now();
            }
        }
        else if(m_useLeftHand && !m_useRightHand)
        {
            if(m_robotLeftHand->isRobotPrepared())
            {
                m_state = HapticGloveFSM::Running;
                m_timePreparation = yarp::os::Time::now();
            }
        }
        else if(!m_useLeftHand && m_useRightHand)
        {
            if(m_robotRightHand->isRobotPrepared())
            {
                m_state = HapticGloveFSM::Running;
            }
        }
        else
        {
            yError()<<"There is not robot hand enabled.";
            return false;
        }

        if(m_state == HapticGloveFSM::Running)
        {
            if(m_useLeftHand)
            {
                if (!m_robotLeftHand->initializeEstimators())
                {
                    yError() << "[HapticGloveModule::updateModule()] cannot initialize the left robot hand.";
                    return false;
                }
            }
            if(m_useRightHand)
            {
                if (!m_robotRightHand->initializeEstimators())
                {
                    yError() << "[HapticGloveModule::updateModule()] cannot initialize the right robot hand.";
                    return false;
                }
            }
        }
    }

    return true;
}

void HapticGloveModule::logData()
{
#ifdef ENABLE_LOGGER
    if (m_enableLogger)
    {
        m_logger->add(m_logger_prefix + "_time", yarp::os::Time::now());


        if (m_useLeftHand)
        {
             /* LEFT HAND */
            /* Robot */
            // Axis
            std::vector<double> icubLeftFingerAxisFeedback, icubLeftFingerAxisReference;
            m_robotLeftHand->getFingerAxisFeedback(icubLeftFingerAxisFeedback);
            m_robotLeftHand->getFingerAxisValueReference(icubLeftFingerAxisReference);

            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisFeedback",
                          icubLeftFingerAxisFeedback);
            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisReference",
                          icubLeftFingerAxisReference);
            if(m_robot=="icub")
            {
                // current
                std::vector<double> icubLeftHandMotorCurrentFeedback, icubLeftHandMotorCurrentReference;
                m_robotLeftHand->getMotorCurrentFeedback(icubLeftHandMotorCurrentFeedback);
                m_robotLeftHand->getMotorCurrentReference(icubLeftHandMotorCurrentReference);

                m_logger->add(m_logger_prefix + "_icubLeftHandMotorCurrnetFeedback",
                              icubLeftHandMotorCurrentFeedback);
                m_logger->add(m_logger_prefix + "_icubLeftHandMotorCurrnetReference",
                              icubLeftHandMotorCurrentReference);

                // pwm
                std::vector<double> icubLeftHandMotorPWMFeedback, icubLeftHandMotorPWMReference;
                m_robotLeftHand->getMotorPwmFeedback(icubLeftHandMotorPWMFeedback);
                m_robotLeftHand->getMotorPwmReference(icubLeftHandMotorPWMReference);

                m_logger->add(m_logger_prefix + "_icubLeftHandMotorPWMFeedback",
                              icubLeftHandMotorPWMFeedback);
                m_logger->add(m_logger_prefix + "_icubLeftHandMotorPWMReference",
                              icubLeftHandMotorPWMReference);
            }

            //pid
            std::vector<double> icubHandPidOutputs;
            m_robotLeftHand->getMotorPidOutputs(icubHandPidOutputs);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorPidOutputs",
                          icubHandPidOutputs);

            std::vector<double> icubLeftFingerAxisVelocityFeedback;
            m_robotLeftHand->getFingerAxisVelocityFeedback(icubLeftFingerAxisVelocityFeedback);
            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisVelocityFeedback",
                          icubLeftFingerAxisVelocityFeedback);

            std::vector<double> icubLeftFingerAxisError(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis()),
                    icubLeftFingerAxisErrorSmoothed(m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());

            for (int i =0; i<m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(); i++)
            {
                icubLeftFingerAxisError[i]=m_icubLeftFingerAxisValueError(i);
                icubLeftFingerAxisErrorSmoothed[i]=m_icubLeftFingerAxisValueErrorSmoothed(i);
            }

            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisError",
                          icubLeftFingerAxisError);
            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisErrorSmoothed",
                          icubLeftFingerAxisErrorSmoothed);
            /* Axis Estimation KF */
            std::vector<double> axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF;
            std::vector<double> axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF;
            Eigen::MatrixXd feedbackAxisCovEstimationKF, referenceAxisCovEstimationKF;


            m_robotLeftHand->getEstimatedMotorsState(axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF, feedbackAxisCovEstimationKF,
                                                     axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF, referenceAxisCovEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisValuesFeedbackKF",
                             axisFeedbackValuesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisVelocitiesFeedbackKF",
                             axisFeedbackVelocitiesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisAccelerationFeedbackKF",
                             axisFeedbackAccelerationEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisCovFeedbackKF",
                             feedbackAxisCovEstimationKF);

            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisValuesReferenceKF",
                             axisReferenceValuesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisVelocitiesReferenceKF",
                             axisReferenceVelocitiesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisAccelerationReferenceKF",
                             axisReferenceAccelerationEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorAxisCovReferenceKF",
                             referenceAxisCovEstimationKF);

            // Joints
            std::vector<double> icubLeftFingerJointsReference, icubLeftFingerJointsFeedback;
            m_robotLeftHand->getFingerJointsFeedback(icubLeftFingerJointsFeedback);
            m_robotLeftHand->getFingerJointReference(icubLeftFingerJointsReference);

            std::vector<double> feedbackJointValuesEstimationKF, expectedJointValuesEstimationKF;
            m_robotLeftHand->getEstimatedJointState(feedbackJointValuesEstimationKF, expectedJointValuesEstimationKF);

            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsReference",
                          icubLeftFingerJointsReference);
            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsFeedback",
                          icubLeftFingerJointsFeedback);
            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsExpectedKF",
                             expectedJointValuesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsFeedbackKF",
                             feedbackJointValuesEstimationKF);

            /* Glove*/
            Eigen::MatrixXd leftHandPose, leftGlovePose, leftHandJointsAngles;
            std::vector<float> leftGloveSensors;
            m_gloveLeftHand->getHandPose(leftHandPose);
            m_gloveLeftHand->getGlovePose(leftGlovePose);
            m_gloveLeftHand->getSensorData(leftGloveSensors);
            m_gloveLeftHand->getHandJointsAngles(leftHandJointsAngles);
            m_logger->add(m_logger_prefix + "_humanLeftHandPose", leftHandPose);
            m_logger->add(m_logger_prefix + "_humanLeftGlovePose", leftGlovePose);
            m_logger->add(m_logger_prefix + "_gloveLeftSensors ", leftGloveSensors);
            std::vector<double> gloveLeftFingerForceFeedback(m_gloveLeftHand->getNoOfForceFeedback());
            for (int i=0;i < m_gloveLeftHand->getNoOfForceFeedback(); i++)
                gloveLeftFingerForceFeedback[i]=m_gloveLeftForceFeedbackReference(i);
            m_logger->add(m_logger_prefix + "_gloveLeftForceFeedback", gloveLeftFingerForceFeedback);

            std::vector<double> imuData;
            m_gloveLeftHand->getGloveIMUData(imuData);
            m_logger->add(m_logger_prefix + "_gloveLeftGloveIMU", imuData);
        }

        if (m_useRightHand)
        {
            /* RIGHT HAND */
            /* Robot */
            // Axis
            std::vector<double> icubRightFingerAxisFeedback, icubRightFingerAxisReference;
            m_robotRightHand->getFingerAxisFeedback(icubRightFingerAxisFeedback);
            m_robotRightHand->getFingerAxisValueReference(icubRightFingerAxisReference);

            m_logger->add(m_logger_prefix + "_icubRightFingerAxisFeedback",
                          icubRightFingerAxisFeedback);
            m_logger->add(m_logger_prefix + "_icubRightFingerAxisReference",
                          icubRightFingerAxisReference);
            if(m_robot=="icub")
            {
                std::vector<double> icubRightHandMotorCurrentFeedback, icubRightHandMotorCurrentReference;
                m_robotRightHand->getMotorCurrentFeedback(icubRightHandMotorCurrentFeedback);
                m_robotRightHand->getMotorCurrentReference(icubRightHandMotorCurrentReference);

                m_logger->add(m_logger_prefix + "_icubRightHandMotorCurrnetFeedback",
                              icubRightHandMotorCurrentFeedback);
                m_logger->add(m_logger_prefix + "_icubRightHandMotorCurrnetReference",
                              icubRightHandMotorCurrentReference);

                std::vector<double> icubRightHandMotorPWMFeedback, icubRightHandMotorPWMReference;
                m_robotRightHand->getMotorPwmFeedback(icubRightHandMotorPWMFeedback);
                m_robotRightHand->getMotorPwmReference(icubRightHandMotorPWMReference);

                m_logger->add(m_logger_prefix + "_icubRightHandMotorPWMFeedback",
                              icubRightHandMotorPWMFeedback);
                m_logger->add(m_logger_prefix + "_icubRightHandMotorPWMReference",
                              icubRightHandMotorPWMReference);

            }
            //pid
            std::vector<double> icubHandPidOutputs;
            m_robotRightHand->getMotorPidOutputs(icubHandPidOutputs);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorPidOutputs",
                          icubHandPidOutputs);

            std::vector<double> icubRightFingerAxisVelocityFeedback;
            m_robotRightHand->getFingerAxisVelocityFeedback(icubRightFingerAxisVelocityFeedback);
            m_logger->add(m_logger_prefix + "_icubRightFingerAxisVelocityFeedback",
                          icubRightFingerAxisVelocityFeedback);


            std::vector<double> icubRightFingerAxisError(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis()),
                    icubRightFingerAxisErrorSmoothed(m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());

            for (int i =0; i<m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(); i++)
            {
                icubRightFingerAxisError[i]=m_icubRightFingerAxisValueError(i);
                icubRightFingerAxisErrorSmoothed[i]=m_icubRightFingerAxisValueErrorSmoothed(i);
            }

            m_logger->add(m_logger_prefix + "_icubRightFingerAxisError",
                          icubRightFingerAxisError);
            m_logger->add(m_logger_prefix + "_icubRightFingerAxisErrorSmoothed",
                          icubRightFingerAxisErrorSmoothed);


            /* Axis Estimation KF */
            std::vector<double> axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF;
            std::vector<double> axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF;
            Eigen::MatrixXd feedbackAxisCovEstimationKF, referenceAxisCovEstimationKF;

            m_robotRightHand->getEstimatedMotorsState(axisFeedbackValuesEstimationKF, axisFeedbackVelocitiesEstimationKF, axisFeedbackAccelerationEstimationKF, feedbackAxisCovEstimationKF,
                                                     axisReferenceValuesEstimationKF, axisReferenceVelocitiesEstimationKF, axisReferenceAccelerationEstimationKF, referenceAxisCovEstimationKF);

            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisValuesFeedbackKF",
                             axisFeedbackValuesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisVelocitiesFeedbackKF",
                             axisFeedbackVelocitiesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisAccelerationFeedbackKF",
                             axisFeedbackAccelerationEstimationKF);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisCovFeedbackKF",
                             feedbackAxisCovEstimationKF);

            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisValuesReferenceKF",
                             axisReferenceValuesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisVelocitiesReferenceKF",
                             axisReferenceVelocitiesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisAccelerationReferenceKF",
                             axisReferenceAccelerationEstimationKF);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorAxisCovReferenceKF",
                             referenceAxisCovEstimationKF);





            // Joints
            std::vector<double> icubRightFingerJointsReference, icubRightFingerJointsFeedback;
            m_robotRightHand->getFingerJointsFeedback(icubRightFingerJointsFeedback);
            m_robotRightHand->getFingerJointReference(icubRightFingerJointsReference);

            std::vector<double> feedbackJointValuesEstimationKF, expectedJointValuesEstimationKF;
            m_robotRightHand->getEstimatedJointState(feedbackJointValuesEstimationKF, expectedJointValuesEstimationKF);

            m_logger->add(m_logger_prefix + "_icubRightFingerJointsReference",
                          icubRightFingerJointsReference);
            m_logger->add(m_logger_prefix + "_icubRightFingerJointsFeedback",
                          icubRightFingerJointsFeedback);
            m_logger->add(m_logger_prefix + "_icubRightFingerJointsExpectedKF",
                             expectedJointValuesEstimationKF);
            m_logger->add(m_logger_prefix + "_icubRightFingerJointsFeedbackKF",
                             feedbackJointValuesEstimationKF);



            /* Glove*/
            Eigen::MatrixXd rightHandPose, rightGlovePose, rightHandJointsAngles;
            std::vector<float> rightGloveSensors;

            m_gloveRightHand->getHandPose(rightHandPose);
            m_gloveRightHand->getGlovePose(rightGlovePose);
            m_gloveRightHand->getSensorData(rightGloveSensors);
            m_gloveRightHand->getHandJointsAngles(rightHandJointsAngles);

            m_logger->add(m_logger_prefix + "_humanRightHandPose", rightHandPose);
            m_logger->add(m_logger_prefix + "_humanRightGlovePose", rightGlovePose);
            m_logger->add(m_logger_prefix + "_gloveRightSensors ", rightGloveSensors);

            std::vector<double> gloveRightFingerForceFeedback(m_gloveRightHand->getNoOfForceFeedback());
            for (int i=0;i < m_gloveRightHand->getNoOfForceFeedback(); i++)
                gloveRightFingerForceFeedback[i]=m_gloveRightForceFeedbackReference(i);
            m_logger->add(m_logger_prefix + "_gloveRightForceFeedback", gloveRightFingerForceFeedback);

            std::vector<double> imuData;
            m_gloveRightHand->getGloveIMUData(imuData);
            m_logger->add(m_logger_prefix + "_gloveRightGloveIMU", imuData);
        }
        m_logger->flush_available_data();
    }
#endif
}
bool HapticGloveModule::openLogger()
{
#ifdef ENABLE_LOGGER
    std::string currentTime = getTimeDateMatExtension();
    std::string fileName = "HapticGloveModule_" + currentTime + "_log.mat";

    yInfo() << "log file name: " << currentTime << fileName;
    m_logger = XBot::MatLogger2::MakeLogger(fileName);
    m_appender = XBot::MatAppender::MakeInstance();
    m_appender->add_logger(m_logger);
    m_appender->start_flush_thread();

    m_logger->create(m_logger_prefix + "_time", 1);

    if(m_useLeftHand){
        // Robot hand axis
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisReference",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisFeedback",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        if(m_robot=="icub")
        {
            m_logger->create(m_logger_prefix + "_icubLeftHandMotorCurrnetFeedback",
                             m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
            m_logger->create(m_logger_prefix + "_icubLeftHandMotorCurrnetReference",
                             m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
            m_logger->create(m_logger_prefix + "_icubLeftHandMotorPWMFeedback",
                             m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
            m_logger->create(m_logger_prefix + "_icubLeftHandMotorPWMReference",
                             m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        }
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorPidOutputs",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());

        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisVelocityFeedback",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());

        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisValuesFeedbackKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisVelocitiesFeedbackKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisAccelerationFeedbackKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisCovFeedbackKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 9); // states: value, velocity, acceleration --> cov matrix size: 3X3=9


        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisValuesReferenceKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisVelocitiesReferenceKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisAccelerationReferenceKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorAxisCovReferenceKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis(), 9);

        // robot axis errors
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisError",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisErrorSmoothed",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedAxis());


        // Robot hand joints
        m_logger->create(m_logger_prefix + "_icubLeftFingerJointsReference",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints());
        m_logger->create(m_logger_prefix + "_icubLeftFingerJointsFeedback",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints());
        m_logger->create(m_logger_prefix + "_icubLeftFingerJointsExpectedKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints());
        m_logger->create(m_logger_prefix + "_icubLeftFingerJointsFeedbackKF",
                         m_robotLeftHand->controlHelper()->getNumberOfActuatedJoints());


        // Human info comming from Glove
        m_logger->create(m_logger_prefix + "_humanLeftHandPose", m_gloveLeftHand->getNoHandLinks(), 7);

        m_logger->create(m_logger_prefix + "_humanLeftGlovePose", m_gloveLeftHand->getNoGloveLinks(), 7);
        m_logger->create(m_logger_prefix + "_gloveLeftSensors", m_gloveLeftHand->getNoSensors());
        m_logger->create(m_logger_prefix + "_gloveLeftForceFeedback", m_gloveLeftHand->getNoOfForceFeedback());
        m_logger->create(m_logger_prefix + "_gloveLeftGloveIMU", 4);


    }
    if(m_useRightHand){
        // Robot hand axis
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisReference",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisFeedback",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        if(m_robot=="icub")
        {
            m_logger->create(m_logger_prefix + "_icubRightHandMotorCurrnetFeedback",
                             m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
            m_logger->create(m_logger_prefix + "_icubRightHandMotorCurrnetReference",
                             m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
            m_logger->create(m_logger_prefix + "_icubRightHandMotorPWMFeedback",
                             m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
            m_logger->create(m_logger_prefix + "_icubRightHandMotorPWMReference",
                             m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        }
        m_logger->create(m_logger_prefix + "_icubRightHandMotorPidOutputs",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());

        m_logger->create(m_logger_prefix + "_icubRightFingerAxisVelocityFeedback",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());

        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisValuesFeedbackKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisVelocitiesFeedbackKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisAccelerationFeedbackKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisCovFeedbackKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), 9);

        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisValuesReferenceKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisVelocitiesReferenceKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisAccelerationReferenceKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorAxisCovReferenceKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis(), 9);

        // robot axis errors
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisError",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisErrorSmoothed",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedAxis());


        // Robot hand joints
        m_logger->create(m_logger_prefix + "_icubRightFingerJointsReference",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedJoints());
        m_logger->create(m_logger_prefix + "_icubRightFingerJointsFeedback",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedJoints());
        m_logger->create(m_logger_prefix + "_icubRightFingerJointsExpectedKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedJoints());
        m_logger->create(m_logger_prefix + "_icubRightFingerJointsFeedbackKF",
                         m_robotRightHand->controlHelper()->getNumberOfActuatedJoints());


        // Human info comming from Glove
        m_logger->create(m_logger_prefix + "_humanRightHandPose", m_gloveRightHand->getNoHandLinks(), 7);

        m_logger->create(m_logger_prefix + "_humanRightGlovePose", m_gloveRightHand->getNoGloveLinks(), 7);
        m_logger->create(m_logger_prefix + "_gloveRightSensors", m_gloveRightHand->getNoSensors());
        m_logger->create(m_logger_prefix + "_gloveRightForceFeedback", m_gloveRightHand->getNoOfForceFeedback());
        m_logger->create(m_logger_prefix + "_gloveRightGloveIMU", 4);

    }
    yInfo() << "[HapticGloveModule::openLogger] Logging is active.";

#else
    yInfo() << "[HapticGloveModule::openLogger] option is not active in CMakeLists.";

#endif
    return true;
}