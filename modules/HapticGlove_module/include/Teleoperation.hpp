// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
#include <RobotSkin.hpp>

// eigen
#include <Eigen/Dense>

// rpc service
#include <thrift/HapticGloveTeleoperationService.h>

namespace HapticGlove
{
class Teleoperation;
struct Data;
} // namespace HapticGlove

/**
 * Data is a data structure to collect all the variables related to the haptic glove teleoperation.
 */
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

    std::vector<double> humanVibrotactileFeedbacks; /// <summary> vibrotactile feedback
                                                    /// vector to the human fingertips using
                                                    /// kinesthetic or skin data

    std::vector<double>
        humanPalmRotation; /// <summary> human palm rotation quaternion vector size: 4x1,

    // skin
    std::vector<double> fingertipsSkinData; /// <summary> fingertips tactile feedbacks values,
                                            /// 60 values read from the robot interface; range: [0,
                                            /// 1]. 0: no laod value, 1 is maximum pressure

    std::vector<double>
        fingertipsCalibratedTactileFeedback; /// <summary> fingertips tactile feedbacks for
                                             /// all the fingerstips: totoal :60 values:
                                             /// fingers thumb, index, middle, ring, little;
                                             /// each finger 12 tactile sensors, range quasi [0, 1]
                                             ///     std::vector<double>

    std::vector<double>
        fingertipsCalibratedDerivativeTactileFeedback; /// <summary> fingertips tactile feedbacks
                                                       /// for all the fingerstips: totoal :60
                                                       /// values: fingers thumb, index, middle,
                                                       /// ring, little; each finger 12 tactile
                                                       /// sensors, range quasi [0, 1]

    std::vector<double> fingercontactStrengthFeedback; /// <summary> fingertips constact strength:
                                                       /// the maximum calibrated tactile sensor
                                                       /// value if the finger is in contact.

    std::vector<double>
        fingercontactStrengthDerivativeFeedback; /// <summary> fingertips constact strength
                                                 /// derivative: the maximum calibrated tactile
                                                 /// sensor value if the finger is in contact.

    std::vector<double>
        robotFingerSkinAbsoluteValueVibrotactileFeedbacks; /// <summary> robot vibrotactile feedback
                                                           /// to human using skin associated with
                                                           /// the absolute tactile values

    std::vector<double>
        robotFingerSkinDerivativeValueVibrotactileFeedbacks; /// <summary> robot vibrotactile
                                                             /// feedback to human using skin
                                                             /// associated with the rate of
                                                             /// change of tactile values

    std::vector<double>
        robotFingerSkinTotalValueVibrotactileFeedbacks; /// <summary> robot vibrotactile feedback to
                                                        /// human using skin associated with the
                                                        /// absolute and differentual tactile values

    std::vector<bool> doRobotFingerSkinsWork; /// <summary> check if the fingertip skins work

    std::vector<bool>
        areFingersSkinInContact; /// <summary> check if the fingertip skins are in contact
};

/**
 * Teleoperation is a class for bi-lateral teleoperation of the haptic glove.
 */
class HapticGlove::Teleoperation : HapticGloveTeleoperationService
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

    bool m_useSkin; /**< the option to give the user the possibility to use the skin data for haptic
                       feedback. */

    double m_timeConfigurationEnd; /**< the moment which the configuration is done */

    std::unique_ptr<RobotController> m_robotController; /**< pointer to the robot controller. */

    std::unique_ptr<GloveControlHelper> m_humanGlove; /**< pointer to the human glove object. */

    std::unique_ptr<Retargeting> m_retargeting; /**< pointer to the human retargeting object. */

    std::unique_ptr<RobotSkin> m_robotSkin; /**< pointer to the robot skin object. */

    double m_calibrationTimePeriod; /**< calibration time period [sec] */

    // Enable at the end
    bool m_enableLogger; /**< log the data (if true) */
    class Logger; /**< forward decleration of the logger class */
    std::unique_ptr<Logger> m_loggerLeftHand; /**< pointer to the logger object. */

    // mutex
    std::mutex m_mutex;

    // RPC port
    yarp::os::Port m_rpcPort;

    virtual bool enableMoveRobot(const bool value) override;

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

    /**
     * Wait the teleoperation before running
     * @return true/false in case of success/failure
     */
    bool wait();

    /**
     * Set the moment in which the configuration is done
     * @param time the time at which the configuration is ended
     */
    void setEndOfConfigurationTime(const double& time);
};

#endif // TELEOPERATION_HPP
