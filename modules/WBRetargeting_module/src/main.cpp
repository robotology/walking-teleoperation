/**
 * @file main.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <OculusModule.hpp>

int main(int argc, char* argv[])
{
    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("wholeBodyRetargeting.ini");

    rf.configure(argc, argv);

    // create the module
    WholeBodyRetargeting module;

    return module.runModule(rf);
}
