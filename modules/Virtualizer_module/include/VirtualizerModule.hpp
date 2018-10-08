/**
 * @file VirtualizerModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef RETARGETING_VIRTUALIZER_MODULE_HPP
#define RETARGETING_VIRTUALIZER_MODULE_HPP

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

#include "CVirt.h"
#include "CVirtDevice.h"

/**
 * RFModule useful to handle the Virtualizere
 */
class VirtualizerModule : public yarp::os::RFModule
{
private:
    double m_dT; /**< RFModule period. */
    double m_deadzone; /**< Value of the deadzone. */
    double m_robotYaw;
	double velocity_factor;
	double oldPlayerYaw;

    yarp::os::RpcClient m_rpcPort; /**< RPC port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_playerOrientationPort; /**< Used to send the player
                                                                          orientation [-pi +pi]. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort;  /**< Used to get the robot
                                                                          orientation. */

    CVirtDevice *m_cvirtDeviceID = nullptr;
    /**
     * Establish the connection with the virtualizer.
     * @return true in case of success and false otherwise.
     */
    bool configureVirtualizer();

    /**
     * Standard threshold function.
     * @param input input
     * @return 0 if the abs(input) < abs(deadzone) otherwise return the input.
     */
    double threshold(const double &input);

public:

    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder &rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;
};

#endif
