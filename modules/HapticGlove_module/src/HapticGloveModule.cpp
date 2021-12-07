/**
 * @file HapticGloveModule.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// std
#include <thread>

// yarp
#include <yarp/os/LogStream.h>

// teleoperation
#include <HapticGloveModule.hpp>
#include <Utils.hpp>

HapticGloveModule::HapticGloveModule()
{
    m_logPrefix = "HapticGloveModule:: ";
};

HapticGloveModule::~HapticGloveModule(){};

bool HapticGloveModule::configure(yarp::os::ResourceFinder& rf)
{
    m_state = HapticGloveFSM::Configuring;

    yarp::os::Value* value;

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
    m_dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asDouble();

    // robot name:
    m_robot = generalOptions.check("robot", yarp::os::Value("icubSim")).asString();

    // check which hand to use
    m_useLeftHand = generalOptions.check("useLeftHand", yarp::os::Value(1)).asBool();
    m_useRightHand = generalOptions.check("useRightHand", yarp::os::Value(1)).asBool();
    yInfo() << m_logPrefix << "use the left hand: " << m_useLeftHand;
    yInfo() << m_logPrefix << "use the right hand: " << m_useRightHand;

    // initialize the left hand teleoperation
    if (m_useLeftHand)
    {
        yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
        leftFingersOptions.append(generalOptions);

        m_leftHand = std::make_unique<HapticGlove::Teleoperation>();
        if (!m_leftHand->configure(leftFingersOptions, getName(), false))
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

        m_rightHand = std::make_unique<HapticGlove::Teleoperation>();
        if (!m_rightHand->configure(rightFingersOptions, getName(), true))
        {
            yError() << m_logPrefix
                     << "unable to initialize the right hand bilateral teleoperation.";
            return false;
        }
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
            yError() << m_logPrefix << "unable to close left hand teleoperation.";
            return false;
        }
    }

    if (m_useRightHand)
    {
        if (!m_rightHand->close())
        {
            yError() << m_logPrefix << "unable to close right hand teleoperation.";
            return false;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 100ms.
    return true;
}

bool HapticGloveModule::updateModule()
{

    if (m_state == HapticGloveFSM::Running)
    {
        double t1 = yarp::os::Time::now();
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
        double t2 = yarp::os::Time::now();
        yInfo() << m_logPrefix << "computation time: " << t2 - t1;

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
            m_state = HapticGloveFSM::Running;
        }
    }

    return true;
}
