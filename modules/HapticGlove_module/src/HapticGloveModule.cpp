// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// std
#include <thread>

// yarp
#include <yarp/os/LogStream.h>

// teleoperation
#include <HapticGloveModule.hpp>
#include <Utils.hpp>

// blf
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

HapticGloveModule::HapticGloveModule()
{
    m_logPrefix = "HapticGloveModule:: ";
};

HapticGloveModule::~HapticGloveModule() = default;

bool HapticGloveModule::configure(yarp::os::ResourceFinder& rf)
{
    m_state = HapticGloveFSM::Configuring;

    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << m_logPrefix << "empty configuration for the HapticGloveModule application.";
        return false;
    }
    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << m_logPrefix << "unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asFloat64();

    // robot name:
    m_robot = generalOptions.check("robot", yarp::os::Value("icubSim")).asString();

    // check which hand to use
    m_useLeftHand = generalOptions.check("useLeftHand", yarp::os::Value(1)).asBool();
    m_useRightHand = generalOptions.check("useRightHand", yarp::os::Value(1)).asBool();
    yInfo() << m_logPrefix << "use the left hand: " << m_useLeftHand;
    yInfo() << m_logPrefix << "use the right hand: " << m_useRightHand;

    m_enableLogger = generalOptions.check("enableLogger", yarp::os::Value(false)).asBool();
    yInfo() << m_logPrefix << "enable logger: " << m_enableLogger;

    // initialize the logger
    if (m_enableLogger)
    {
        auto loggerOption
            = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(rf)
                  ->getGroup("LOGGER")
                  .lock();
        if (loggerOption == nullptr)
        {
            yError() << "[WalkingModule::configure] Unable to get the group LOGGER.";
            return false;
        }

        std::string logPort;
        if (!loggerOption->getParameter("remote", logPort))
        {
            yError() << m_logPrefix  << "Unable to get the remote from the group LOGGER.";
            return false;
        }

        // prepend the module name to the port name
        logPort = "/" + getName() + logPort;
        loggerOption->setParameter("remote", logPort);
        if (!m_vectorsCollectionServer.initialize(loggerOption))
        {
            yError() << m_logPrefix << "Unable to get the string from searchable.";
            return false;
        }
    }

    // initialize the left hand teleoperation
    if (m_useLeftHand)
    {
        yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
        leftFingersOptions.append(generalOptions);
        if (m_enableLogger)
        {
            yarp::os::Bottle& loggerOptions = rf.findGroup("LOGGER");
            leftFingersOptions.append(loggerOptions);
        }

        m_leftHand = std::make_unique<HapticGlove::Teleoperation>();
        if (!m_leftHand->configure(leftFingersOptions, m_robot, false, m_vectorsCollectionServer))
        {
            yError() << m_logPrefix
                     << "unable to initialize the left hand bilateral teleoperation.";
            return false;
        }
    }

    // initialize the right hand teleoperation
    if (m_useRightHand)
    {
        yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
        rightFingersOptions.append(generalOptions);

        if (m_enableLogger)
        {
            yarp::os::Bottle& loggerOptions = rf.findGroup("LOGGER");
            rightFingersOptions.append(loggerOptions);
        }

        m_rightHand = std::make_unique<HapticGlove::Teleoperation>();
        if (!m_rightHand->configure(rightFingersOptions, m_robot, true, m_vectorsCollectionServer))
        {
            yError() << m_logPrefix
                     << "unable to initialize the right hand bilateral teleoperation.";
            return false;
        }
    }

    if (m_enableLogger)
    {
        // The two teleoperation objects fill the metadata internally
        m_vectorsCollectionServer.finalizeMetadata();
    }

    // wainting time after preparation and before running state machine
    m_waitingStartTime = 0;
    m_waitingDurationTime
        = generalOptions.check("waitingDurationTime", yarp::os::Value(5.0)).asFloat64();

    // update the end of the configuration time step
    double timeConfigurationEnd = yarp::os::Time::now();

    if (m_useLeftHand)
    {
        m_leftHand->setEndOfConfigurationTime(timeConfigurationEnd);
    }

    if (m_useRightHand)
    {
        m_rightHand->setEndOfConfigurationTime(timeConfigurationEnd);
    }

    yInfo() << m_logPrefix << "configuration is done. ";
    m_state = HapticGloveFSM::Preparing;

    return true;
}

double HapticGloveModule::getPeriod()
{
    return m_dT;
}

bool HapticGloveModule::close()
{
    yInfo() << m_logPrefix << "trying to close.";
    if (m_useLeftHand)
    {
        if (!m_leftHand->close())
        {
            yWarning() << m_logPrefix << "unable to close left hand teleoperation.";
        }
    }

    if (m_useRightHand)
    {
        if (!m_rightHand->close())
        {
            yWarning() << m_logPrefix << "unable to close right hand teleoperation.";
        }
    }

    return true;
}

bool HapticGloveModule::updateModule()
{
    double t1 = yarp::os::Time::now();

    if (m_state == HapticGloveFSM::Running)
    {
        if (m_enableLogger)
        {
            m_vectorsCollectionServer.prepareData();
            m_vectorsCollectionServer.clearData();
        }

        if (m_useLeftHand)
        {
            if (!m_leftHand->run())
            {
                yError() << m_logPrefix << "cannot run the left hand.";
                return false;
            }
        }
        if (m_useRightHand)
        {
            if (!m_rightHand->run())
            {
                yError() << m_logPrefix << "cannot run the left hand.";
                return false;
            }
        }

        if (m_enableLogger)
        {
            m_vectorsCollectionServer.sendData();
        }

    } else if (m_state == HapticGloveFSM::Configuring)
    {
        // do nothing at the moment, it should be configured before
        yError() << m_logPrefix << "haptic glove is in configuring status, but it should not be.";
        return false;

    } else if (m_state == HapticGloveFSM::Preparing)
    {

        bool isPrepared = true;
        if (m_useLeftHand)
        {
            bool tmp;
            if (!m_leftHand->prepare(tmp))
            {
                yWarning() << m_logPrefix << "unable to prepare the left hand.";
            }
            isPrepared &= tmp;
        }

        if (m_useRightHand)
        {
            bool tmp;
            if (!m_rightHand->prepare(tmp))
            {
                yWarning() << m_logPrefix << "unable to prepare the right hand.";
            }
            isPrepared &= tmp;
        }

        if (isPrepared)
        {
            m_state = HapticGloveFSM::Waiting;
            m_waitingStartTime = yarp::os::Time::now();
        }
    } else if (m_state == HapticGloveFSM::Waiting)
    {
        double timeNow = yarp::os::Time::now();

        if (timeNow - m_waitingStartTime > m_waitingDurationTime)
        {
            m_state = HapticGloveFSM::Running;
        } else
        {
            if (m_useLeftHand)
            {
                if (!m_leftHand->wait())
                {
                    yWarning() << m_logPrefix << "the left hand is unable to wait.";
                }
            }

            if (m_useRightHand)
            {
                if (!m_rightHand->wait())
                {
                    yWarning() << m_logPrefix << "the right hand is unable to wait.";
                }
            }
        }
    }
    double t2 = yarp::os::Time::now();

    return true;
}
