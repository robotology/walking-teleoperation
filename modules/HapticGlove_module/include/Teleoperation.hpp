/**
 * @file Teleoperation.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef TELEOPERATION_HPP
#define TELEOPERATION_HPP

// yarp
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

// teleoperation
#include <GloveControlHelper.hpp>
#include <Retargeting.hpp>
#include <RobotController.hpp>

// eigen
#include <Eigen/Dense>

namespace HapticGlove
{
class Teleoperation;
struct Data;
} // namespace HapticGlove

struct HapticGlove::Data
{
    double time; /// <summary> the current time

    // robot
    std::vector<double> robotAxisReferences; /// <summary> robot axis reference vector
    std::vector<double> robotAxisFeedbacks; /// <summary> robot axis feedback vector
    std::vector<double> robotAxisVelocityFeedbacks; /// <summary> robot velocity feedback vector

    std::vector<double> robotJointReferences; /// <summary> robot joint reference vector
    std::vector<double> robotJointFeedbacks; /// <summary> robot joint feedback vector

    std::vector<double> robotAxisValueErrors; /// <summary> robot axis value error vector
    std::vector<double> robotAxisVelocityErrors; /// <summary> robot axis velocity error vector

    std::vector<double>
        robotMotorCurrentReferences; /// <summary> robot motor current reference vector
    std::vector<double>
        robotMotorCurrentFeedbacks; /// <summary> robot motor current feedback vector

    std::vector<double> robotMotorPwmReferences; /// <summary> robot motor pwm reference
    std::vector<double> robotMotorPwmFeedbacks; /// <summary> robot motor pwm feedback vector

    std::vector<double> robotMotorPidOutputs; /// <summary> robot motor pid output

    std::vector<double>
        robotAxisValueReferencesKf; /// <summary> robot axis value reference vector computed by KF
    std::vector<double> robotAxisVelocityReferencesKf; /// <summary> robot axis velocity reference
                                                       /// vector computed by KF
    std::vector<double> robotAxisAccelerationReferencesKf; /// <summary> robot axis acceleration
                                                           /// reference vector computed by KF
    Eigen::MatrixXd
        robotAxisCovReferencesKf; /// <summary> robot axis covariance reference vector
                                  /// computed by KF, size: numRobotActuatedAxis x 9 (9: 3x3)

    std::vector<double>
        robotAxisValueFeedbacksKf; /// <summary> robot axis value feedback vector computed by KF
    std::vector<double> robotAxisVelocityFeedbacksKf; /// <summary> robot axis velocity feedback
                                                      /// vector computed by KF
    std::vector<double> robotAxisAccelerationFeedbacksKf; /// <summary> robot axis acceleration
                                                          /// feedback vector computed by KF
    Eigen::MatrixXd
        robotAxisCovFeedbacksKf; /// <summary> robot axis covariance feedback vector computed by
                                 /// KF, size: numRobotActuatedAxis x 9 (9: 3x3)

    std::vector<double>
        robotJointsExpectedKf; /// <summary> robot joint expected value computed by KF (expect
                               /// value= axis feedback value * aixs-joint coupling value)

    std::vector<double>
        robotJointsFeedbackKf; /// <summary> robot joint feedback value computed by KF

    // human
    std::vector<double> humanJointValues; /// <summary> juman joint values
    Eigen::MatrixXd humanFingertipPoses; /// <summary> human fingertip poses (size: number of hand
                                         /// fingertips (i.e., 5) x 7)
    std::vector<double>
        humanForceFeedbacks; /// <summary> force feedback vector to the human fingertips
    std::vector<double> humanVibrotactileFeedbacks; /// <summary> vibrotactile feedback vector to
                                                    /// the human fingertips
    std::vector<double>
        humanPalmRotation; /// <summary> human palm rotation quaternion vector size: 4x1,
};

class HapticGlove::Teleoperation
{
    std::string m_logPrefix; /**< log prefix */

    double m_dT; /**< module period. */

    std::string m_robot; /**< robot name. */

    bool m_getHumanMotionRange; /**< get the motion range of the human joints, i.e., at the start up
                                   human should move the hand fingers such that reaching the joint
                                   extremums*/

    Data m_data; /**< data structure. */

    bool m_moveRobot; /**< the option to give the user the possibility to move (or not) the robot
                         (default value: true)*/

    double m_timeConfigurationEnd; /**< the moment which the configuration is done */

    std::unique_ptr<RobotController> m_robotController; /**< pointer to the robot controller. */

    std::unique_ptr<GloveControlHelper> m_humanGlove; /**< pointer to the human glove object. */

    std::unique_ptr<Retargeting> m_retargeting; /**< pointer to the human retargeting object. */

    double m_calibrationTimePeriod; /**< calibration time period [sec] */

    // Enable at the end
    bool m_enableLogger; /**< log the data (if true) */
    class Logger; /**< forward decleration of the logger class */
    std::unique_ptr<Logger> m_loggerLeftHand; /**< pointer to the logger object. */

    /**
     * Get all the feedback signal from the robot controller
     * @return true/false in case of success/failure
     */
    bool getFeedbacks();

public:
    /**
     * Constructor
     */
    Teleoperation();

    /**
     * Destructor
     * */
    ~Teleoperation();

    /**
     * Configure the bilteral teleoperation class implementation
     * @param config configuration options
     * @param name name of the robot
     * @param rightHand if true the right hand is used
     * @return true/false in case of success/failure
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Close the teleoperation class.
     * @return true/false in case of success/failure
     */
    bool close();

    /**
     * run the bilateral teleoperation policy
     * @return true/false in case of success/failure
     */
    bool run();

    /**
     * Prepare the teleoperation at the startup, i.e., human and robot perform the calibration step
     * @param isPrepared check if the preparation is successfully performed
     * @return true/false in case of success/failure
     */
    bool prepare(bool& isPrepared);
};

#endif // TELEOPERATION_HPP
