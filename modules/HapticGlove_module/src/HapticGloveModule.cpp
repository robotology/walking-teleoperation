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
    yInfo() << "fingers axis: " << m_icubLeftFingerAxisFeedback.toString();
    yInfo() << "fingers joints: " << m_icubLeftFingerJointsFeedback.toString();

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

        // 2- Compute the reference values for the haptic glove, including resistance force and
        // vibrotactile feedback

        // 3- Set the reference joint valued for the iCub hand fingers
        // left hand
        //        m_leftHandFingers->setFingersAxisReference(m_icubLeftFingerAxisReference);
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
            std::cerr << "110 \n";
            m_logger->flush_available_data();
        }
#endif

    } else if (m_state == HapticGloveFSM::Configured)
    {
        // TODO
        m_timeConfigurationStarting = yarp::os::Time::now();
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

    m_logger->create(m_logger_prefix + "_loc_joypad_x_y",
                     2); // [x,y] component for robot locomotion
    yInfo() << "[HapticGloveModule::openLogger] Logging is active.";
#else
    yInfo() << "[HapticGloveModule::openLogger] option is not active in CMakeLists.";

#endif
    return true;
}
