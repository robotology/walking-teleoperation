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

#include <mutex>

#include <deque>

// YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>

#include <CVirt.h>
#include <CVirtDevice.h>

#include <thrift/VirtualizerCommands.h>

/**
 * RFModule useful to handle the Virtualizer
 */
class VirtualizerModule : public yarp::os::RFModule, public VirtualizerCommands
{
private:
    double m_dT; /**< RFModule period. */
    double m_deadzone; /**< Value of the deadzone. */
    double m_robotYaw; /**<Robot orientation. */
    double m_scale_X, m_scale_Y; /**< Linear and angular velocity scaling factor */
    double m_oldPlayerYaw; /**< Player orientation (coming from the virtualizer) retrieved at the
                              previous time step. */

    yarp::os::Port
        m_rpcServerPort; /**< Port used to send command to the virtualizer application. */

    yarp::os::RpcClient m_rpcPort; /**< RPC port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_playerOrientationPort; /**< Used to send the player
                                                                          orientation [-pi +pi]. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort; /**< Used to get the robot
                                                                         orientation. */

    std::mutex m_mutex;
    CybSDK::VirtDevice* m_cvirtDeviceID = nullptr;

    bool m_useRingVelocity;
    unsigned int m_movingAverageWindowSize;
    double m_velocityDeadzone;
    double m_velocityScaling;
    std::deque<double> m_movingAverage;

    bool m_useHeadForTurning;
    yarp::dev::PolyDriver m_headDevice; /**< Device to retrieve neck values. */
    yarp::dev::IEncoders* m_encodersInterface{nullptr}; /**< Encoders interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    int m_neckYawAxisIndex;
    bool m_yawAxisPointsUp;
    double m_neckYawScaling;
    double m_neckYawDeadzone;
    double m_isMovingDeadzone;


    /**
     * Establish the connection with the virtualizer.
     * @return true in case of success and false otherwise.
     */
    bool configureVirtualizer();

    bool configureRingVelocity(const yarp::os::Bottle& ringVelocityGroup);

    bool configureHeadControl(const yarp::os::Bottle& headControlGroup);

    /**
     * Standard threshold function.
     * @param input input
     * @return 0 if the abs(input) < abs(deadzone) otherwise return the input.
     */
    double threshold(const double& input);

    /**
     * Standard threshold function.
     * @param input input
     * @param deadzone The deadzone value
     * @return 0 if the abs(input) < abs(deadzone) otherwise return the input.
     */
    double threshold(const double& input, double deadzone);

    double filteredRingVelocity(double newVelocity);

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
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    /**
     * Reset the player orientation
     */
    void resetPlayerOrientation() override;
};

#endif
