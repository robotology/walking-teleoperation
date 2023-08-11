// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
#include <yarp/dev/IFrameTransform.h>

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
    double m_angleThresholdOperatorStill; /**< Angle threshold to consider the operator still. */
    double m_angleThresholdOperatorMoving; /**< Angle threshold to consider the operator moving. */
    double m_operatorCurrentStillAngle; /**< The angle in which the operator when considered still the first time. */
    double m_operatorStillTime; /**< First instant in which the operator was considered still. */
    double m_operatorStillTimeThreshold; /**< Time threshold to conder the operator still. */
    bool m_operatorMoving; /**< The operator is currently moving. */
    bool m_useVelocitySignOnly; /**< Use only the speed sign for rotating. */
    double m_jammedMovingTime; /**< The duration with which the output is kept constant after the operator starts moving */
    double m_jammedStartTime; /**< Start time in which the lateral output is jammed. */
    double m_jammedMovingRobotAngle; /**< The angle variation that the robot has to do when the operator starts moving. */
    double m_jammedRobotStartAngle; /**< The robot angle when the operator starts moving. */
    bool m_jammedRobotStartAngleValid; /**< Check if the robot angle was valid when the operator started moving. */
    double m_jammedValue; /**< The jammed output for the lateral direction. **/
    bool m_isJammed; /**< True if the lateral output is jammed. **/
    bool m_jammedRobotOnce; /**< True if it jammed already once. **/


    bool m_useHeadForTurning; /**< Flag to use the head for controlling the robot turning while walking. */
    yarp::dev::PolyDriver m_headDevice; /**< Device to retrieve neck values. */
    yarp::dev::IEncoders* m_encodersInterface{nullptr}; /**< Encoders interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    int m_neckYawAxisIndex; /**< The index of the neck yaw angle. */
    bool m_yawAxisPointsUp; /**< Flag for the direction of the neck yaw axis. */
    double m_neckYawScaling; /**< Scaling from the neck yaw value to the desired point for the unicycle (the neck yaw is in degrees). */
    double m_neckYawDeadzone; /**< Value below which the neck is considered straight (the value is in degrees). */
    bool m_useOnlyHeadForTurning; /**< Flag to use only the head to control the direction while walking. */

    yarp::dev::PolyDriver m_tfDriver; /**< Device to publish the Virtualizer transform. */
    yarp::dev::IFrameTransform* m_tfPublisher; /**< Interface to publish and retrieve transforms. */
    std::string m_tfRootFrame; /**< The name of the root frame name. */
    std::string m_tfFrameName; /**< The name of the virtualizer frame in the transform server. */
    bool m_useTf; /**< Flag to check if the transform server needs to be used. */
    yarp::sig::Matrix m_tfMatrix; /**< Buffer to publish the transform. */

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
     * @brief Configure the TransformServer interface
     * @param tfGroup The group containing the parameters for the TransformServer interface
     * @return True if successfull.
     */
    bool configureTransformServer(const yarp::os::Bottle& tfGroup);

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
     * Standard sign function.
     * @param input input
     * @param deadzone The deadzone value
     * @return 0 if the abs(input) < abs(deadzone) otherwise return the sign of the input.
     */
    double sign(const double& input, double deadzone);

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

    /**
     * @brief Reads the current robot yaw
     * @return True if the update succeded. False otherwise.
     */
    bool updateRobotYaw();

    /**
     * @brief Get the current player yaw measured from the virtualizer
     * @return The player yaw
     */
    double getPlayerYaw();

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

    /**
     * Reset the player height
     */
    virtual void resetPlayerHeight() override;

     /**
     * Reset the player still angle
     */
    virtual void forceStillAngle() override;
};

#endif
