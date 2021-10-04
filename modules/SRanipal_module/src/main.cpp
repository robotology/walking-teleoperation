/**
 * @file main.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <SRanipalModule.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

int main(int argc, char* argv[])
{

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.configure(argc, argv);

    SRanipalModule sranipalModule;

    return sranipalModule.runModule(rf);
}
