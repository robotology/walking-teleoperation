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

    m_state = HapticGloveFSM::Configured;

    return true;
}

double HapticGloveModule::getPeriod()
{
    return m_dT;
}

bool HapticGloveModule::close()
{
    return true;
}

bool HapticGloveModule::evaluateRobotFingersReferences()
{
    return true;
}

bool HapticGloveModule::getFeedbacks()
{

    return true;
}

bool HapticGloveModule::updateModule()
{
    if (!getFeedbacks())
    {
        yError() << "[HapticGloveModule::updateModule] Unable to get the feedback";
        return false;
    }

    if (m_state == HapticGloveFSM::Running)
    {
        // TODO

    } else if (m_state == HapticGloveFSM::Configured)
    {
        // TODO
    } else if (m_state == HapticGloveFSM::InPreparation)
    {
        // TODO
    }
    // TO

    return true;
}
