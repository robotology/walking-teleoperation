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

    // configure fingers retargeting
    m_leftHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
    leftFingersOptions.append(generalOptions);
    if (!m_leftHandFingers->configure(leftFingersOptions, getName()))
    {
        yError()
            << "[HapticGloveModule::configure] Unable to initialize the left fingers retargeting.";
        return false;
    }

    m_rightHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
    rightFingersOptions.append(generalOptions);
    if (!m_rightHandFingers->configure(rightFingersOptions, getName()))
    {
        yError()
            << "[HapticGloveModule::configure] Unable to initialize the right fingers retargeting.";
        return false;
    }

    m_gloveRightHand = std::make_unique<HapticGlove::GloveControlHelper>();
    if (!m_gloveRightHand->configure(generalOptions, getName(), true))
    {
        yError() << "[HapticGloveModule::configure] Unable to initialize the right glove control "
                    "helper.";
        return false;
    }

    m_gloveLeftHand = std::make_unique<HapticGlove::GloveControlHelper>();
    if (!m_gloveLeftHand->configure(generalOptions, getName(), false))
    {
        yError() << "[HapticGloveModule::configure] Unable to initialize the right glove control "
                    "helper.";
        return false;
    }

    m_gloveRightBuzzMotorReference.resize(m_gloveRightHand->getNoOfBuzzMotors(), 0.0);

    m_timePreparationStarting = 0.0;
    m_timeNow = 0.0;
    m_timeConfigurationStarting = 0.0;

    m_icubLeftFingerAxisReference.resize(m_leftHandFingers->controlHelper()->getActuatedDoFs());
    m_icubLeftFingerAxisFeedback.resize(m_leftHandFingers->controlHelper()->getActuatedDoFs());
    m_icubLeftFingerJointsReference.resize(m_leftHandFingers->controlHelper()->getNumberOfJoints());
    m_icubLeftFingerJointsFeedback.resize(m_leftHandFingers->controlHelper()->getNumberOfJoints());

    m_icubRightFingerAxisReference.resize(m_rightHandFingers->controlHelper()->getActuatedDoFs());
    m_icubRightFingerAxisFeedback.resize(m_rightHandFingers->controlHelper()->getActuatedDoFs());
    m_icubRightFingerJointsReference.resize(
        m_rightHandFingers->controlHelper()->getNumberOfJoints());
    m_icubRightFingerJointsFeedback.resize(
        m_rightHandFingers->controlHelper()->getNumberOfJoints());

    // open the logger only if all the vecotos sizes are clear.
    if (m_enableLogger)
    {
        if (!openLogger())
        {
            yError() << "[HapticGloveModule::configure] Unable to open the logger";
            return false;
        }
    }

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
    m_gloveRightHand->stopFeedback();
    m_gloveRightHand->turnOffBuzzMotors();

    m_gloveLeftHand->stopFeedback();
    m_gloveLeftHand->turnOffBuzzMotors();


    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // wait for 10ms.
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

    if (!m_leftHandFingers->updateFeedback())
    {
        yError() << "[HapticGloveModule::getFeedbacks()] unable to update the feedback values of "
                    "the left hand fingers.";
    }
    m_leftHandFingers->getFingerAxisFeedback(m_icubLeftFingerAxisFeedback);
    m_leftHandFingers->getFingerJointsFeedback(m_icubLeftFingerJointsFeedback);
    // yInfo() << "fingers axis: " << m_icubLeftFingerAxisFeedback.toString();
    // yInfo() << "fingers joints: " << m_icubLeftFingerJointsFeedback.toString();

    return true;
}

bool HapticGloveModule::updateModule()
{
    yInfo() << "********************************* [HapticGloveModule::updateModule()] "
               "*********************************";
    if (!getFeedbacks())
    {
        yError() << "[HapticGloveModule::updateModule] Unable to get the feedback";
        return false;
    }

    if (m_state == HapticGloveFSM::Running)
    {
        m_timeNow = yarp::os::Time::now();

        // 1- Compute the reference values for the iCub hand joint fingers
        const unsigned noLeftFingersAxis = m_leftHandFingers->controlHelper()->getActuatedDoFs();
        const unsigned noLeftFingersJoints
            = m_leftHandFingers->controlHelper()->getNumberOfJoints();

        for (unsigned i = 0; i < noLeftFingersJoints; i++)
        {
            //            m_icubLeftFingerAxisReference(i)
            m_icubLeftFingerJointsReference(i)
                = M_PI_4 + M_PI_4 * sin((m_timeNow - m_timePreparationStarting) - M_PI_2);
            //            m_icubRightFingerAxisReference(i) = m_icubLeftFingerAxisReference(i);
            //            m_icubLeftFingerJointsReference(i) = m_icubLeftFingerAxisReference(i);
        }
        double tmp_buzzVal;
        if ((m_timeNow - m_timePreparationStarting) < (50 * M_PI))
        {
            yInfo() << "give buzz ref value higher than zero, time: "
                    << m_timeNow - m_timePreparationStarting;
            tmp_buzzVal = 85;
            // 0 + 20.0 * sin((m_timeNow - m_timePreparationStarting) / 4.0 - M_PI_2);
        } else
        {
            yInfo() << "give buzz ref value equal to zero, time: "
                    << m_timeNow - m_timePreparationStarting;
            tmp_buzzVal = 0.0;
        }
        for (int i = 0; i < 5; i++)
            m_gloveRightBuzzMotorReference(i) = tmp_buzzVal;
        // m_gloveRightBuzzMotorReference(1) = tmp_buzzVal;

        // m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);

                    // m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);
        // m_gloveRightHand->setFingersForceReference(m_gloveRightBuzzMotorReference);

        
        if (false){
        if (m_timeNow - m_timePreparationStarting < 10.0)
        {
            // m_timePreparationStarting = m_timeNow;
            yInfo() << "++++++++++++++++++++++++ time: " << m_timeNow - m_timePreparationStarting
                    << " give force feedback command";
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
                    << m_timeNow - m_timePreparationStarting << " stop command";

             m_gloveRightHand->stopFeedback();
            m_timePreparationStarting = m_timeNow;
        }
        }
        // if (((int)tmp_buzzVal)%4==0)
        //    m_gloveRightHand->setPalmFeedback(0);

        Eigen::MatrixXd rightHandPose, rightGlovePose, rightHandJointsAngles;
        std::vector<float> rightGloveSensors, leftGloveSensors;
        m_gloveRightHand->getHandPose(rightHandPose);
        m_gloveRightHand->getGlovePose(rightGlovePose);
        m_gloveRightHand->getSensorData(rightGloveSensors);
        m_gloveRightHand->getHandJointsAngles(rightHandJointsAngles);

        Eigen::MatrixXd leftHandPose, leftGlovePose, leftHandJointsAngles;

        m_gloveLeftHand->getHandPose(leftHandPose);
        m_gloveLeftHand->getGlovePose(leftGlovePose);
        m_gloveLeftHand->getSensorData(leftGloveSensors);
        m_gloveLeftHand->getHandJointsAngles(leftHandJointsAngles);


        yInfo() << "**************";
        //yInfo() << "gloveSensors: \n" << gloveSensors;
        //yInfo() << "**************";
        //std::cout << "hand pose: \n" << handPose << std::endl;
        //yInfo() << "**************";
        //std::cout << "glove pose: \n" << glovePose << std::endl;
        //yInfo() << "handJointsAngles.size :" << rightHandJointsAngles.rows() << rightHandJointsAngles.cols();
        //yInfo() << "index finger, first joint: " << rightHandJointsAngles(4, 0)
        //        << rightHandJointsAngles(4, 1) << rightHandJointsAngles(4, 2);

        //yInfo() << "index finger, second joint: " << rightHandJointsAngles(5, 0)
        //        << rightHandJointsAngles(5, 1) << rightHandJointsAngles(5, 2);

        //yInfo() << "index finger, third joint: " << rightHandJointsAngles(6, 0)
        //        << rightHandJointsAngles(6, 1) << rightHandJointsAngles(6, 2);

        //yInfo() << "index finger, fourth joint: " << rightHandJointsAngles(7, 0)
        //        << rightHandJointsAngles(7, 1) << rightHandJointsAngles(7, 2);

        //std::cout << "handJointsAngles: \n" << rightHandJointsAngles << std::endl;

        // yInfo() << "handJointsAngles.size :" << leftHandJointsAngles.rows() <<
        //        leftHandJointsAngles.cols();
        //yInfo() << "index finger, first joint: " << leftHandJointsAngles(4, 0)
        //         << leftHandJointsAngles(4, 1) << leftHandJointsAngles(4, 2);

        // yInfo() << "index finger, second joint: " << leftHandJointsAngles(5, 0)
        //        << leftHandJointsAngles(5, 1) << leftHandJointsAngles(5, 2);

        // yInfo() << "index finger, third joint: " << leftHandJointsAngles(6, 0)
        //         << leftHandJointsAngles(6, 1) << leftHandJointsAngles(6, 2);

        // yInfo() << "index finger, fourth joint: " << leftHandJointsAngles(7, 0)
        //         << leftHandJointsAngles(7, 1) << leftHandJointsAngles(7, 2);

         std::cout << "handJointsAngles: \n" << leftHandJointsAngles << std::endl;





        // 2- Compute the reference values for the haptic glove, including resistance force and
        // vibrotactile feedback

        // 3- Set the reference joint valued for the iCub hand fingers
        // left hand
        //        m_leftHandFingers->setFingersAxisReference(m_icubLeftFingerAxisReference);
         
         {
             m_icubLeftFingerJointsReference.zero();
         
         int i = 0;
         // thumb: (i:i+2)
         m_icubLeftFingerJointsReference(i + 0) = -leftHandJointsAngles(0, 2);
         m_icubLeftFingerJointsReference(i + 1) = leftHandJointsAngles(1, 1);
         m_icubLeftFingerJointsReference(i + 2) = leftHandJointsAngles(2, 1);

         //index (i+3:i+5)
         m_icubLeftFingerJointsReference(i + 3) = leftHandJointsAngles(4, 1);
         m_icubLeftFingerJointsReference(i + 4) = leftHandJointsAngles(5, 1);
         m_icubLeftFingerJointsReference(i + 5) = leftHandJointsAngles(6, 1);
         
         //middle (i+6:i+8)
         m_icubLeftFingerJointsReference(i + 6) = leftHandJointsAngles(8, 1);
         m_icubLeftFingerJointsReference(i + 7) = leftHandJointsAngles(9, 1);
         m_icubLeftFingerJointsReference(i + 8) = leftHandJointsAngles(10, 1);

         //ring (i+9:i+11)
         m_icubLeftFingerJointsReference(i + 9) =  leftHandJointsAngles(12, 1);
         m_icubLeftFingerJointsReference(i + 10) = leftHandJointsAngles(13, 1);
         m_icubLeftFingerJointsReference(i + 11) = leftHandJointsAngles(14, 1);

         //pinkie (i+12:i+14)
         m_icubLeftFingerJointsReference(i + 12) = leftHandJointsAngles(16, 1);
         m_icubLeftFingerJointsReference(i + 13) = leftHandJointsAngles(17, 1);
         m_icubLeftFingerJointsReference(i + 14) = leftHandJointsAngles(18, 1);
         }
         yInfo() << "m_icubLeftFingerJointsReference: " << m_icubLeftFingerJointsReference.toString();

        m_leftHandFingers->setFingersJointReference(m_icubLeftFingerJointsReference);
        m_leftHandFingers->move();

        // right hand
        //        m_rightHandFingers->setFingersAxisReference(m_icubRightFingerAxisReference);
        //        m_rightHandFingers->move();

        // 4- Set the reference values for the haptic glove, including resistance force and
        // vibrotactile feedback

#ifdef ENABLE_LOGGER
        if (m_enableLogger)
        {
            m_logger->add(m_logger_prefix + "_time", yarp::os::Time::now());

            /* Left Hand */
            // Axis
            std::vector<double> icubLeftFingerAxisFeedback, icubLeftFingerAxisReference;
            m_leftHandFingers->getFingerAxisFeedback(icubLeftFingerAxisFeedback);
            m_leftHandFingers->getFingerAxisReference(icubLeftFingerAxisReference);

            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisFeedback",
                          icubLeftFingerAxisFeedback);
            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisReference",
                          icubLeftFingerAxisReference);
            // Joints
            std::vector<double> icubLeftFingerJointsFeedback, icubLeftFingerJointsReference;
            m_leftHandFingers->getFingerJointsFeedback(icubLeftFingerJointsFeedback);
            m_leftHandFingers->getFingerJointReference(icubLeftFingerJointsReference);

            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsFeedback",
                          icubLeftFingerJointsFeedback);
            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsReference",
                          icubLeftFingerJointsReference);

            /* Right Hand */
            // Axis
            std::vector<double> icubRightFingerAxisFeedback, icubRightFingerAxisReference;
            m_rightHandFingers->getFingerAxisFeedback(icubRightFingerAxisFeedback);
            m_rightHandFingers->getFingerAxisReference(icubRightFingerAxisReference);

            m_logger->add(m_logger_prefix + "_icubRightFingerAxisFeedback",
                          icubRightFingerAxisFeedback);
            m_logger->add(m_logger_prefix + "_icubRightFingerAxisReference",
                          icubRightFingerAxisReference);
            // Joints
            std::vector<double> icubRightFingerJointsReference, icubRightFingerJointsFeedback;
            m_rightHandFingers->getFingerJointsFeedback(icubRightFingerJointsFeedback);
            m_rightHandFingers->getFingerJointReference(icubRightFingerJointsReference);

            m_logger->add(m_logger_prefix + "_icubRightFingerJointsReference",
                          icubRightFingerJointsReference);
            m_logger->add(m_logger_prefix + "_icubRightFingerJointsFeedback",
                          icubRightFingerJointsFeedback);

            // GLOVE: based on data worn by human user
            m_logger->add(m_logger_prefix + "_humanHandPose", handPose);
            m_logger->add(m_logger_prefix + "_humanGlovePose", glovePose);
            m_logger->add(m_logger_prefix + "_gloveSensors ", );

            , glovePose;

            std::cerr << "110 \n";
            m_logger->flush_available_data();
        }
#endif

    } else if (m_state == HapticGloveFSM::Configured)
    {

        // TODO
        m_timeConfigurationStarting = yarp::os::Time::now();
        if (!m_gloveRightHand->setupGlove())
        {
            yError() << "[HapticGloveModule::updateModule()] cannot setup the right hand glove.";
            return false;
        }
        if (!m_gloveLeftHand->setupGlove())
        {
            yError() << "[HapticGloveModule::updateModule()] cannot setup the left hand glove.";
            return false;
        }
        m_state = HapticGloveFSM::InPreparation;
    } else if (m_state == HapticGloveFSM::InPreparation)
    {

        m_timePreparationStarting = yarp::os::Time::now();

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
                << m_timePreparationStarting - m_timeConfigurationStarting;
        int dTime = int((m_timePreparationStarting - m_timeConfigurationStarting) / m_dT);
        int axisNumber = dTime / 100;
        if (axisNumber >= m_leftHandFingers->controlHelper()->getActuatedDoFs())
        {
            yInfo() << "******>>>> Data collected ...";
            if (!m_leftHandFingers->trainCouplingMatrix())
            {
                yInfo() << "cannot claibrate the coupling matrix and find the coefficient matrix";
                return false;
            }
            m_timePreparationStarting = yarp::os::Time::now();
            m_state = HapticGloveFSM::Running;

        } else
        {
            double time = double(dTime % 100) / 100.0 * (M_PI * 2.0);
            m_leftHandFingers->LogDataToCalibrateRobotMotorsJointsCouplingSin(time, axisNumber);
        }
    }
    // TO

    return true;
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

    m_logger->create(m_logger_prefix + "_icubLeftFingerAxisReference",
                     m_leftHandFingers->controlHelper()->getActuatedDoFs());
    m_logger->create(m_logger_prefix + "_icubLeftFingerAxisFeedback",
                     m_leftHandFingers->controlHelper()->getActuatedDoFs());
    m_logger->create(m_logger_prefix + "_icubRightFingerAxisReference",
                     m_rightHandFingers->controlHelper()->getActuatedDoFs());
    m_logger->create(m_logger_prefix + "_icubRightFingerAxisFeedback",
                     m_rightHandFingers->controlHelper()->getActuatedDoFs());

    m_logger->create(m_logger_prefix + "_icubLeftFingerJointsReference",
                     m_leftHandFingers->controlHelper()->getNumberOfJoints());
    m_logger->create(m_logger_prefix + "_icubLeftFingerJointsFeedback",
                     m_leftHandFingers->controlHelper()->getNumberOfJoints());
    m_logger->create(m_logger_prefix + "_icubRightFingerJointsReference",
                     m_rightHandFingers->controlHelper()->getNumberOfJoints());
    m_logger->create(m_logger_prefix + "_icubRightFingerJointsFeedback",
                     m_rightHandFingers->controlHelper()->getNumberOfJoints());

    m_logger->create(m_logger_prefix + "_humanHandPose", m_gloveRightHand->getNoHandLinks(), 7);

    m_logger->create(m_logger_prefix + "_humanGlovePose", m_gloveRightHand->getNoGloveLinks(), 7);
    m_logger->create(m_logger_prefix + "_gloveSensors", m_gloveRightHand->getNoSensors(), 7);

    m_logger->create(m_logger_prefix + "_loc_joypad_x_y",
                     2); // [x,y] component for robot locomotion
    yInfo() << "[HapticGloveModule::openLogger] Logging is active.";
#else
    yInfo() << "[HapticGloveModule::openLogger] option is not active in CMakeLists.";

#endif
    return true;
}
