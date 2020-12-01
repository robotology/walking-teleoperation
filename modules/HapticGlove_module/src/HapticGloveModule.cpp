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

    // check which hand to use
    m_useLeftHand = generalOptions.check("useLeftHand", yarp::os::Value(1)).asBool();
    m_useRightHand = generalOptions.check("useRightHand", yarp::os::Value(1)).asBool();
    yInfo() << "[HapticGloveModule::configure] use the left hand: " << m_useLeftHand;
    yInfo() << "[HapticGloveModule::configure] use the right hand: " << m_useRightHand;

    // configure fingers retargeting
    if (m_useLeftHand)
    {
        // initialize the retargeting class of the left hand
        m_robotLeftHand = std::make_unique<RobotController>();
        yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
        leftFingersOptions.append(generalOptions);
        if (!m_robotLeftHand->configure(leftFingersOptions, getName()))
        {
            yError() << "[HapticGloveModule::configure] Unable to initialize the left fingers "
                        "retargeting.";
            return false;
        }

        // intialize the glove control helper of the left hand
        m_gloveLeftHand = std::make_unique<HapticGlove::GloveControlHelper>();
        if (!m_gloveLeftHand->configure(generalOptions, getName(), false))
        {
            yError()
                    << "[HapticGloveModule::configure] Unable to initialize the right glove control "
                       "helper.";
            return false;
        }
    }
    if (m_useRightHand)
    {
        // initialize the retargeting class of the right hand
        m_robotRightHand = std::make_unique<RobotController>();
        yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
        rightFingersOptions.append(generalOptions);
        if (!m_robotRightHand->configure(rightFingersOptions, getName()))
        {
            yError() << "[HapticGloveModule::configure] Unable to initialize the right fingers "
                        "retargeting.";
            return false;
        }

        // intialize the glove control helper of the right hand
        m_gloveRightHand = std::make_unique<HapticGlove::GloveControlHelper>();
        if (!m_gloveRightHand->configure(generalOptions, getName(), true))
        {
            yError()
                    << "[HapticGloveModule::configure] Unable to initialize the right glove control "
                       "helper.";
            return false;
        }
    }
    std::cerr << "conf 05 \n";

    if (m_useLeftHand)
    {
        m_icubLeftFingerAxisReference.resize(m_robotLeftHand->controlHelper()->getActuatedDoFs());
        m_icubLeftFingerAxisFeedback.resize(m_robotLeftHand->controlHelper()->getActuatedDoFs());
        m_icubLeftFingerJointsReference.resize(
                    m_robotLeftHand->controlHelper()->getNumberOfJoints());
        m_icubLeftFingerJointsFeedback.resize(
                    m_robotLeftHand->controlHelper()->getNumberOfJoints());

        m_gloveLeftBuzzMotorReference.resize(m_gloveLeftHand->getNoOfBuzzMotors(), 0.0);
        m_gloveLeftForceFeedbackReference.resize(m_gloveLeftHand->getNoOfForceFeedback(), 0.0);

        m_axisErrorLeftSmoother
                = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_robotLeftHand->controlHelper()->getActuatedDoFs(), m_dT, smoothingTime);
        yarp::sig::Vector buff(m_robotLeftHand->controlHelper()->getActuatedDoFs(), 0.0);

        m_axisErrorLeftSmoother->init(buff);

        //Axis
        m_icubLeftFingerAxisError.resize(m_robotLeftHand->controlHelper()->getActuatedDoFs(),0.0);
        m_icubLeftFingerAxisErrorSmoothed.resize(m_robotLeftHand->controlHelper()->getActuatedDoFs(),0.0);

    }
    std::cerr << "conf 06 \n";
    if (m_useRightHand)
    {
        m_icubRightFingerAxisReference.resize(
                    m_robotRightHand->controlHelper()->getActuatedDoFs());
        m_icubRightFingerAxisFeedback.resize(
                    m_robotRightHand->controlHelper()->getActuatedDoFs());
        m_icubRightFingerJointsReference.resize(
                    m_robotRightHand->controlHelper()->getNumberOfJoints());
        m_icubRightFingerJointsFeedback.resize(
                    m_robotRightHand->controlHelper()->getNumberOfJoints());

        m_gloveRightBuzzMotorReference.resize(m_gloveRightHand->getNoOfBuzzMotors(), 0.0);
        m_gloveRightForceFeedbackReference.resize(m_gloveRightHand->getNoOfForceFeedback(), 0.0);

        m_axisErrorRightSmoother
                = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_robotRightHand->controlHelper()->getActuatedDoFs(), m_dT, smoothingTime);
        yarp::sig::Vector buff(m_robotRightHand->controlHelper()->getActuatedDoFs(), 0.0);

        m_axisErrorRightSmoother->init(buff);

        //Axis
        m_icubRightFingerAxisError.resize(m_robotRightHand->controlHelper()->getActuatedDoFs(),0.0);
        m_icubRightFingerAxisErrorSmoothed.resize(m_robotRightHand->controlHelper()->getActuatedDoFs(),0.0);

    }

    m_timePreparationStarting = 0.0;
    m_timeNow = 0.0;
    m_timeConfigurationStarting = 0.0;
    std::cerr << "conf 07 \n";
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
        // get feedback from the robot left hand values
        if (!m_robotLeftHand->updateFeedback())
        {
            yError()
                    << "[HapticGloveModule::getFeedbacks()] unable to update the feedback values of "
                       "the left hand fingers.";
        }
        m_robotLeftHand->getFingerAxisFeedback(m_icubLeftFingerAxisFeedback);
        m_robotLeftHand->getFingerAxisValueReference(m_icubLeftFingerAxisReference);

        m_robotLeftHand->getFingerJointsFeedback(m_icubLeftFingerJointsFeedback);
        // yInfo() << "left fingers axis: " << m_icubLeftFingerAxisFeedback.toString();
        // yInfo() << "left fingers joints: " << m_icubLeftFingerJointsFeedback.toString();
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
        m_robotRightHand->getFingerAxisFeedback(m_icubRightFingerAxisFeedback);
        m_robotRightHand->getFingerAxisValueReference(m_icubRightFingerAxisReference);

        m_robotRightHand->getFingerJointsFeedback(m_icubRightFingerJointsFeedback);
        yInfo() << "right fingers axis feedback: " << m_icubRightFingerAxisFeedback.toString();
        yInfo() << "right fingers axis reference: " << m_icubRightFingerAxisReference.toString();
        yInfo() << "right fingers joints: " << m_icubRightFingerJointsFeedback.toString();
    }

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
        yInfo() << " >>> STATUS: Running";

        m_timeNow = yarp::os::Time::now();
        std::cerr << "Debug: 01 \n";
        if(false)
        {
            if (m_useLeftHand)
            {
                // 1- Compute the reference values for the iCub hand joint fingers
                const unsigned noLeftFingersAxis
                        = m_robotLeftHand->controlHelper()->getActuatedDoFs();
                const unsigned noLeftFingersJoints
                        = m_robotLeftHand->controlHelper()->getNumberOfJoints();

                std::cerr << "Debug: 02 \n";
                for (unsigned i = 0; i < noLeftFingersJoints; i++)
                {
                    //            m_icubLeftFingerAxisReference(i)
                    m_icubLeftFingerJointsReference(i)
                            = M_PI_4 + M_PI_4 * sin((m_timeNow - m_timePreparationStarting) - M_PI_2);
                    //            m_icubRightFingerAxisReference(i) = m_icubLeftFingerAxisReference(i);
                    //            m_icubLeftFingerJointsReference(i) = m_icubLeftFingerAxisReference(i);
                }
            }
            std::cerr << "Debug: 03 \n";
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
            std::cerr << "Debug: 04 \n";
            for (int i = 0; i < 5; i++)
                m_gloveRightBuzzMotorReference(i) = tmp_buzzVal;
            // m_gloveRightBuzzMotorReference(1) = tmp_buzzVal;

            // m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);

            // m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);
            // m_gloveRightHand->setFingersForceReference(m_gloveRightBuzzMotorReference);
        }
        std::cerr << "Debug: 05 \n";
        if (false)
        {
            std::cerr << "Debug: 06 \n";
            if (m_timeNow - m_timePreparationStarting < 10.0)
            {
                // m_timePreparationStarting = m_timeNow;
                yInfo() << "++++++++++++++++++++++++ time: "
                        << m_timeNow - m_timePreparationStarting << " give force feedback command";
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
        if (m_useLeftHand)
        {
            std::cerr << "Debug: 07 \n";
            Eigen::MatrixXd leftHandPose, leftGlovePose, leftHandJointsAngles;
            std::vector<float> leftGloveSensors;

            m_gloveLeftHand->getHandPose(leftHandPose);
            m_gloveLeftHand->getGlovePose(leftGlovePose);
            m_gloveLeftHand->getSensorData(leftGloveSensors);
            m_gloveLeftHand->getHandJointsAngles(leftHandJointsAngles);

            std::cout << "handJointsAngles: \n" << leftHandJointsAngles << std::endl;

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

            std::cout << "right handJointsAngles: \n" << rightHandJointsAngles << std::endl;

            m_icubRightFingerJointsReference.zero();

            int i = 0;
            /* iff only index
            // index (i+3:i+5)
            m_icubRightFingerJointsReference(i + 0) = rightHandJointsAngles(4, 1);
            m_icubRightFingerJointsReference(i + 1) = rightHandJointsAngles(5, 1);
            m_icubRightFingerJointsReference(i + 2) = rightHandJointsAngles(6, 1);
            */
            /*
            // thumb + index
            m_icubRightFingerJointsReference(i + 0) = rightHandJointsAngles(1, 1);
            m_icubRightFingerJointsReference(i + 1) = rightHandJointsAngles(2, 1);
            m_icubRightFingerJointsReference(i + 2) = rightHandJointsAngles(4, 1);
            m_icubRightFingerJointsReference(i + 3) = rightHandJointsAngles(5, 1);
            m_icubRightFingerJointsReference(i + 4) = rightHandJointsAngles(6, 1);
            */
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

            // pinkie (i+12:i+14)
            m_icubRightFingerJointsReference(i + 12) = rightHandJointsAngles(16, 1);
            m_icubRightFingerJointsReference(i + 13) = rightHandJointsAngles(17, 1);
            m_icubRightFingerJointsReference(i + 14) = rightHandJointsAngles(18, 1);
            yInfo() << "m_icubRightFingerJointsReference: "
                    << m_icubRightFingerJointsReference.toString();
        }

        /*** COMPUTE FORCE FEEDBACK***/
        if (m_useLeftHand)
        {
            //            std::vector<double> icubRightFingerAxisFeedback, icubRightFingerAxisReference;


            for (int i=0; i<m_icubLeftFingerAxisReference.size();i++)
            {
                m_icubLeftFingerAxisError(i)= std::abs(m_icubLeftFingerAxisFeedback(i) - m_icubLeftFingerAxisReference(i));
            }

            m_axisErrorLeftSmoother->computeNextValues(m_icubLeftFingerAxisError);
            m_icubLeftFingerAxisErrorSmoothed = m_axisErrorLeftSmoother->getPos();

            // "r_thumb_oppose":0 , "r_thumb_proximal":1, "r_thumb_distal":2, "r_index_proximal":3,
            // "r_index-distal":4, "r_middle-proximal":5, "r_middle-distal":6, "r_little-fingers":7

            yInfo()<<"Left Axis thumb: "<<m_icubLeftFingerAxisErrorSmoothed(2)<<" , "
                  << m_icubLeftFingerAxisErrorSmoothed(1)<<" , " <<
                     m_icubLeftFingerAxisErrorSmoothed(0);
            yInfo()<<"Left Axis index: "<<m_icubLeftFingerAxisErrorSmoothed(4)<< " , "
                  << m_icubLeftFingerAxisErrorSmoothed(3);
            yInfo()<<"Left Axis middle: "<<m_icubLeftFingerAxisErrorSmoothed(6)<< " , "
                  << m_icubLeftFingerAxisErrorSmoothed(5);
            yInfo()<<"Left Axis springy: "<<m_icubLeftFingerAxisErrorSmoothed(7);

            int k_gain=150;
            //            for (int i = 0; i < m_gloveRightForceFeedbackReference.size(); i++)
            m_gloveLeftForceFeedbackReference(0)=150*(m_icubLeftFingerAxisErrorSmoothed(2)
                                                      + m_icubLeftFingerAxisErrorSmoothed(1));
            //                    + std::abs(icubRightFingerAxisFeedback[0]-icubRightFingerAxisReference[0])
            m_gloveLeftForceFeedbackReference(1)=150*(m_icubLeftFingerAxisErrorSmoothed(4)
                                                      + m_icubLeftFingerAxisErrorSmoothed(3));
            m_gloveLeftForceFeedbackReference(2)=50*(m_icubLeftFingerAxisErrorSmoothed(6)
                                                     + m_icubLeftFingerAxisErrorSmoothed(5));
            m_gloveLeftForceFeedbackReference(3)=100*(m_icubLeftFingerAxisErrorSmoothed(7));
            m_gloveLeftForceFeedbackReference(4)=100*std::abs(m_icubLeftFingerAxisErrorSmoothed(7));

            yInfo()<<"Hand Feedback Finger 0: "<<m_gloveLeftForceFeedbackReference(0);
            yInfo()<<"Hand Feedback Finger 1: "<<m_gloveLeftForceFeedbackReference(1);
            yInfo()<<"Hand Feedback Finger 2: "<<m_gloveLeftForceFeedbackReference(2);
            yInfo()<<"Hand Feedback Finger 3: "<<m_gloveLeftForceFeedbackReference(3);
            yInfo()<<"Hand Feedback Finger 4: "<<m_gloveLeftForceFeedbackReference(4);

            for (int i = 0; i < 5; i++)
                m_gloveLeftBuzzMotorReference(i) = m_gloveLeftForceFeedbackReference(i);


        }
        if (m_useRightHand)
        {
            //            std::vector<double> icubRightFingerAxisFeedback, icubRightFingerAxisReference;


            for (int i=0; i<m_icubRightFingerAxisReference.size();i++)
            {
                m_icubRightFingerAxisError(i)= std::abs(m_icubRightFingerAxisFeedback(i) - m_icubRightFingerAxisReference(i));
            }

            m_axisErrorRightSmoother->computeNextValues(m_icubRightFingerAxisError);
            m_icubRightFingerAxisErrorSmoothed = m_axisErrorRightSmoother->getPos();

            // "r_thumb_oppose":0 , "r_thumb_proximal":1, "r_thumb_distal":2, "r_index_proximal":3,
            // "r_index-distal":4, "r_middle-proximal":5, "r_middle-distal":6, "r_little-fingers":7

            yInfo()<<"Right Axis thumb: "<<m_icubRightFingerAxisErrorSmoothed(2)<<" , "
                  << m_icubRightFingerAxisErrorSmoothed(1)<<" , " <<
                     m_icubRightFingerAxisErrorSmoothed(0);
            yInfo()<<"Right Axis index: "<<m_icubRightFingerAxisErrorSmoothed(4)<< " , "
                  << m_icubRightFingerAxisErrorSmoothed(3);
            yInfo()<<"Right Axis middle: "<<m_icubRightFingerAxisErrorSmoothed(6)<< " , "
                  << m_icubRightFingerAxisErrorSmoothed(5);
            yInfo()<<"Right Axis springy: "<<m_icubRightFingerAxisErrorSmoothed(7);

            int k_gain=150;
            //            for (int i = 0; i < m_gloveRightForceFeedbackReference.size(); i++)
            m_gloveRightForceFeedbackReference(0)=150*(m_icubRightFingerAxisErrorSmoothed(2)
                                                       + m_icubRightFingerAxisErrorSmoothed(1));
            //                    + std::abs(icubRightFingerAxisFeedback[0]-icubRightFingerAxisReference[0])
            m_gloveRightForceFeedbackReference(1)=150*(m_icubRightFingerAxisErrorSmoothed(4)
                                                       + m_icubRightFingerAxisErrorSmoothed(3));
            m_gloveRightForceFeedbackReference(2)=50*(m_icubRightFingerAxisErrorSmoothed(6)
                                                      + m_icubRightFingerAxisErrorSmoothed(5));
            m_gloveRightForceFeedbackReference(3)=100*(m_icubRightFingerAxisErrorSmoothed(7));
            m_gloveRightForceFeedbackReference(4)=100*std::abs(m_icubRightFingerAxisErrorSmoothed(7));

            yInfo()<<"Hand Feedback Finger 0: "<<m_gloveRightForceFeedbackReference(0);
            yInfo()<<"Hand Feedback Finger 1: "<<m_gloveRightForceFeedbackReference(1);
            yInfo()<<"Hand Feedback Finger 2: "<<m_gloveRightForceFeedbackReference(2);
            yInfo()<<"Hand Feedback Finger 3: "<<m_gloveRightForceFeedbackReference(3);
            yInfo()<<"Hand Feedback Finger 4: "<<m_gloveRightForceFeedbackReference(4);

            for (int i = 0; i < 5; i++)
                m_gloveRightBuzzMotorReference(i) = m_gloveRightForceFeedbackReference(i);



        }

        if (m_useLeftHand)
        {
            // set robot values
            m_robotLeftHand->setFingersJointReference(m_icubLeftFingerJointsReference);
            m_robotLeftHand->move();

            // set glove values
            m_gloveLeftHand->setFingersForceReference(m_gloveLeftForceFeedbackReference);
            m_gloveLeftHand->setBuzzMotorsReference(m_gloveLeftBuzzMotorReference);
        }

        if (m_useRightHand)
        {
            // set robot values
            m_robotRightHand->setFingersJointReference(m_icubRightFingerJointsReference);
            m_robotRightHand->move();

            // set glove values
            m_gloveRightHand->setFingersForceReference(m_gloveRightForceFeedbackReference);
            m_gloveRightHand->setBuzzMotorsReference(m_gloveRightBuzzMotorReference);
        }

        // right hand
        //        m_rightHandFingers->setFingersAxisReference(m_icubRightFingerAxisReference);
        //        m_rightHandFingers->move();

        // 4- Set the reference values for the haptic glove, including resistance force and
        // vibrotactile feedback

        logData();

        // yInfo() << "[HapticGloveModule::updateModule] <FSM::InPreparation> exiting ....";
        // exit(0);

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

        m_timeConfigurationStarting = yarp::os::Time::now();
        m_state = HapticGloveFSM::InPreparation;
    } else if (m_state == HapticGloveFSM::InPreparation)
    {
        yInfo() << " >>> STATUS: InPreparation";

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
        int axisNumber = dTime / 500;
        if (m_useLeftHand)
        {
            if (axisNumber >= m_robotLeftHand->controlHelper()->getActuatedDoFs())
            {
                yInfo() << "******>>>> Data collected ...";
                if (!m_robotLeftHand->trainCouplingMatrix())
                {
                    yInfo()
                            << "cannot claibrate the coupling matrix and find the coefficient matrix";
                    return false;
                }
                m_timePreparationStarting = yarp::os::Time::now();
                m_state = HapticGloveFSM::Running;

            } else
            {
                double time = double(dTime % 500) / 500.0 * (M_PI * 2.0);
                m_robotLeftHand->LogDataToCalibrateRobotMotorsJointsCouplingSin(time, axisNumber);
            }
        }

        if (m_useRightHand)
        {
            if (axisNumber >= m_robotRightHand->controlHelper()->getActuatedDoFs())
            {
                yInfo() << "******>>>> Data collected ...";
                if (!m_robotRightHand->trainCouplingMatrix())
                {
                    yInfo()
                            << "cannot claibrate the coupling matrix and find the coefficient matrix";
                    return false;
                }
                m_timePreparationStarting = yarp::os::Time::now();
                m_state = HapticGloveFSM::Running;

            } else
            {
                double time = double(dTime % 500) / 500.0 * (M_PI * 2.0);
                m_robotRightHand->LogDataToCalibrateRobotMotorsJointsCouplingSin(time,
                                                                                 axisNumber);
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

            std::vector<double> icubLeftHandMotorCurrentFeedback, icubLeftHandMotorCurrentReference;
            m_robotLeftHand->getMotorCurrentFeedback(icubLeftHandMotorCurrentFeedback);
            m_robotLeftHand->getMotorCurrentReference(icubLeftHandMotorCurrentReference);

            m_logger->add(m_logger_prefix + "icubLeftHandMotorCurrnetFeedback",
                          icubLeftHandMotorCurrentFeedback);
            m_logger->add(m_logger_prefix + "_icubLeftHandMotorCurrnetReference",
                          icubLeftHandMotorCurrentReference);

            std::vector<double> icubLeftFingerAxisVelocityFeedback;
            m_robotLeftHand->getFingerAxisVelocityFeedback(icubLeftFingerAxisVelocityFeedback);
            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisVelocityFeedback",
                          icubLeftFingerAxisVelocityFeedback);


            std::vector<double> icubLeftFingerAxisError(m_robotLeftHand->controlHelper()->getActuatedDoFs()),
                    icubLeftFingerAxisErrorSmoothed(m_robotLeftHand->controlHelper()->getActuatedDoFs());

            for (int i =0; i<m_robotLeftHand->controlHelper()->getActuatedDoFs(); i++)
            {
                icubLeftFingerAxisError[i]=m_icubLeftFingerAxisError(i);
                icubLeftFingerAxisErrorSmoothed[i]=m_icubLeftFingerAxisErrorSmoothed(i);
            }

            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisError",
                          icubLeftFingerAxisError);
            m_logger->add(m_logger_prefix + "_icubLeftFingerAxisErrorSmoothed",
                          icubLeftFingerAxisErrorSmoothed);



            // Joints
            std::vector<double> icubLeftFingerJointsReference, icubLeftFingerJointsFeedback;
            m_robotLeftHand->getFingerJointsFeedback(icubLeftFingerJointsFeedback);
            m_robotLeftHand->getFingerJointReference(icubLeftFingerJointsReference);

            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsReference",
                          icubLeftFingerJointsReference);
            m_logger->add(m_logger_prefix + "_icubLeftFingerJointsFeedback",
                          icubLeftFingerJointsFeedback);
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

            std::vector<double> icubRightHandMotorCurrentFeedback, icubRightHandMotorCurrentReference;
            m_robotRightHand->getMotorCurrentFeedback(icubRightHandMotorCurrentFeedback);
            m_robotRightHand->getMotorCurrentReference(icubRightHandMotorCurrentReference);

            m_logger->add(m_logger_prefix + "icubRightHandMotorCurrnetFeedback",
                          icubRightHandMotorCurrentFeedback);
            m_logger->add(m_logger_prefix + "_icubRightHandMotorCurrnetReference",
                          icubRightHandMotorCurrentReference);

            std::vector<double> icubRightFingerAxisVelocityFeedback;
            m_robotRightHand->getFingerAxisVelocityFeedback(icubRightFingerAxisVelocityFeedback);
            m_logger->add(m_logger_prefix + "_icubRightFingerAxisVelocityFeedback",
                          icubRightFingerAxisVelocityFeedback);


            std::vector<double> icubRightFingerAxisError(m_robotRightHand->controlHelper()->getActuatedDoFs()),
                    icubRightFingerAxisErrorSmoothed(m_robotRightHand->controlHelper()->getActuatedDoFs());

            for (int i =0; i<m_robotRightHand->controlHelper()->getActuatedDoFs(); i++)
            {
                icubRightFingerAxisError[i]=m_icubRightFingerAxisError(i);
                icubRightFingerAxisErrorSmoothed[i]=m_icubRightFingerAxisErrorSmoothed(i);
            }

            m_logger->add(m_logger_prefix + "_icubRightFingerAxisError",
                          icubRightFingerAxisError);
            m_logger->add(m_logger_prefix + "_icubRightFingerAxisErrorSmoothed",
                          icubRightFingerAxisErrorSmoothed);



            // Joints
            std::vector<double> icubRightFingerJointsReference, icubRightFingerJointsFeedback;
            m_robotRightHand->getFingerJointsFeedback(icubRightFingerJointsFeedback);
            m_robotRightHand->getFingerJointReference(icubRightFingerJointsReference);

            m_logger->add(m_logger_prefix + "_icubRightFingerJointsReference",
                          icubRightFingerJointsReference);
            m_logger->add(m_logger_prefix + "_icubRightFingerJointsFeedback",
                          icubRightFingerJointsFeedback);
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
    std::cerr << "log 01 \n";

    m_logger->create(m_logger_prefix + "_time", 1);
    std::cerr << "log 01-1 \n";

    if(m_useLeftHand){
        // Robot hand axis
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisReference",
                         m_robotLeftHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisFeedback",
                         m_robotLeftHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorCurrnetFeedback",
                         m_robotLeftHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubLeftHandMotorCurrnetReference",
                         m_robotLeftHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisVelocityFeedback",
                         m_robotLeftHand->controlHelper()->getActuatedDoFs());


        // robot axis errors
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisError",
                         m_robotLeftHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubLeftFingerAxisErrorSmoothed",
                         m_robotLeftHand->controlHelper()->getActuatedDoFs());


        // Robot hand joints
        m_logger->create(m_logger_prefix + "_icubLeftFingerJointsReference",
                         m_robotLeftHand->controlHelper()->getNumberOfJoints());
        m_logger->create(m_logger_prefix + "_icubLeftFingerJointsFeedback",
                         m_robotLeftHand->controlHelper()->getNumberOfJoints());


        // Human info comming from Glove
        m_logger->create(m_logger_prefix + "_humanLeftHandPose", m_gloveLeftHand->getNoHandLinks(), 7);

        m_logger->create(m_logger_prefix + "_humanLeftGlovePose", m_gloveLeftHand->getNoGloveLinks(), 7);
        m_logger->create(m_logger_prefix + "_gloveLeftSensors", m_gloveLeftHand->getNoSensors());
        m_logger->create(m_logger_prefix + "_gloveLeftForceFeedback", m_gloveLeftHand->getNoOfForceFeedback());


    }
    std::cerr << "log 01-3 \n";
    if(m_useRightHand){
        // Robot hand axis
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisReference",
                         m_robotRightHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisFeedback",
                         m_robotRightHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorCurrnetFeedback",
                         m_robotRightHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubRightHandMotorCurrnetReference",
                         m_robotRightHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisVelocityFeedback",
                         m_robotRightHand->controlHelper()->getActuatedDoFs());


        // robot axis errors
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisError",
                         m_robotRightHand->controlHelper()->getActuatedDoFs());
        m_logger->create(m_logger_prefix + "_icubRightFingerAxisErrorSmoothed",
                         m_robotRightHand->controlHelper()->getActuatedDoFs());


        // Robot hand joints
        m_logger->create(m_logger_prefix + "_icubRightFingerJointsReference",
                         m_robotRightHand->controlHelper()->getNumberOfJoints());
        m_logger->create(m_logger_prefix + "_icubRightFingerJointsFeedback",
                         m_robotRightHand->controlHelper()->getNumberOfJoints());


        // Human info comming from Glove
        m_logger->create(m_logger_prefix + "_humanRightHandPose", m_gloveRightHand->getNoHandLinks(), 7);

        m_logger->create(m_logger_prefix + "_humanRightGlovePose", m_gloveRightHand->getNoGloveLinks(), 7);
        m_logger->create(m_logger_prefix + "_gloveRightSensors", m_gloveRightHand->getNoSensors());
        m_logger->create(m_logger_prefix + "_gloveRightForceFeedback", m_gloveRightHand->getNoOfForceFeedback());


    }
    yInfo() << "[HapticGloveModule::openLogger] Logging is active.";

#else
    yInfo() << "[HapticGloveModule::openLogger] option is not active in CMakeLists.";

#endif
    return true;
}
