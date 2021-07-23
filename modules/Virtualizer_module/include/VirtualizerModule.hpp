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
    double m_angleDeadzone; /**< Value of the deadzone. */
    double m_speedDeadzone; /**< Value below which the person is considered still, hence avoiding to send a reference to the robot. */
    double m_robotYaw; /**< Robot orientation. */
    double m_scale_X, m_scale_Y; /**< Linear and angular velocity scaling factor */
    double m_oldPlayerYaw; /**< Player orientation (coming from the virtualizer) retrieved at the
                              previous time step. */

    yarp::os::Port
        m_rpcServerPort; /**< Port used to send command to the virtualizer application. */

    yarp::os::BufferedPort<yarp::sig::Vector> m_robotGoalPort; /**< Port used to specify the desired goal position. */

    yarp::os::BufferedPort<yarp::sig::Vector> m_playerOrientationPort; /**< Used to send the player
                                                                          orientation [-pi +pi]. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort; /**< Used to get the robot
                                                                         orientation. */

    std::mutex m_mutex; /**< Internal mutex. */
    CybSDK::VirtDevice* m_cvirtDeviceID = nullptr; /**< Virtualizer device. */

    bool m_useRingVelocity;  /**< Flag to use the ring velocity instead of the position error. */
    unsigned int m_movingAverageWindowSize; /**< Window size for the moving average that filters the ring velocity. */
    double m_velocityDeadzone; /**< Absolute value below which the ring is considered still. */
    double m_velocityScaling; /**< Scaling value from the encoder value to a reference point. */
    std::deque<double> m_movingAverage; /**< Buffer to save velocity data. */

    bool m_useHeadForTurning; /**< Flag to use the head for controlling the robot turning while walking. */
    yarp::dev::PolyDriver m_headDevice; /**< Device to retrieve neck values. */
    yarp::dev::IEncoders* m_encodersInterface{nullptr}; /**< Encoders interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    int m_neckYawAxisIndex; /**< The index of the neck yaw angle. */
    bool m_yawAxisPointsUp; /**< Flag for the direction of the neck yaw axis. */
    double m_neckYawScaling; /**< Scaling from the neck yaw value to the desired point for the unicycle (the neck yaw is in degrees). */
    double m_neckYawDeadzone; /**< Value below which the neck is considered straight (the value is in degrees). */
    bool m_useOnlyHeadForTurning; /**< Flag to use only the head to control the direction while walking. */

    /**
     * Establish the connection with the virtualizer.
     * @return true in case of success and false otherwise.
     */
    bool configureVirtualizer();

    /**
     * @brief Configure the parameters relative to the ring velocity estimation
     * @param ringVelocityGroup The group containing the ring velocity parameters.
     * @return True if successfull.
     */
    bool configureRingVelocity(const yarp::os::Bottle& ringVelocityGroup);

    /**
     * @brief Configure the parameters relative to the use of the neck yaw to control the walking direction
     * @param ringVelocityGroup The group containing the head control parameters.
     * @return True if successfull.
     */
    bool configureHeadControl(const yarp::os::Bottle& headControlGroup);

    /**
     * Standard threshold function.
     * @param input input
     * @param deadzone The deadzone value
     * @return 0 if the abs(input) < abs(deadzone) otherwise return the input.
     */
    double threshold(const double& input, double deadzone);

    /**
     * @brief Filter the ring velocity
     * @param newVelocity The new velocity to be considered
     * @return The filtered velocity
     */
    double filteredRingVelocity(double newVelocity);

    /**
     * @brief Check if the neck is working fine.
     * @return True if the neck is working fine, false otherwise.
     */
    bool isNeckWorking();

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
