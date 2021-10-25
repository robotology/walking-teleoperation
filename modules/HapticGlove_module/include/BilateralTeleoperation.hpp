#ifndef BILATERALTELEOPERATION_HPP
#define BILATERALTELEOPERATION_HPP

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
#include <RobotController_hapticGlove.hpp>

namespace HapticGlove
{
class BilateralTeleoperation;
}

class HapticGlove::BilateralTeleoperation
{
    std::string m_logPrefix;

    double m_dT; /**< module period. */

    bool m_getHumanMotionRange; /**< get the motion range of the human joints*/

    yarp::sig::Vector m_icubLeftFingerAxisValueReference;
    yarp::sig::Vector m_icubLeftFingerAxisValueFeedback;
    yarp::sig::Vector m_icubLeftFingerAxisVelocityReference;
    yarp::sig::Vector m_icubLeftFingerAxisVelocityFeedback;
    std::vector<double> m_icubLeftFingerJointsReference;
    std::vector<double> m_icubLeftFingerJointsFeedback;

    std::vector<double> m_gloveLeftBuzzMotorReference;
    std::vector<double> m_gloveLeftForceFeedbackReference;

    double m_timePreparation;
    double m_timeConfigurationEnding;
    double m_timeNow;

    std::vector<double> m_icubLeftFingerAxisValueError;

    std::vector<double> m_icubLeftFingerAxisVelocityError;

    yarp::sig::Vector m_leftTotalGain;
    yarp::sig::Vector m_leftVelocityGain;
    yarp::sig::Vector m_leftBuzzMotorsGain;

    std::vector<double> m_axisFeedbackValuesEstimationKF;
    std::vector<double> m_axisFeedbackVelocitiesEstimationKF;
    std::vector<double> m_axisFeedbackAccelerationEstimationKF;
    std::vector<double> m_axisReferenceValuesEstimationKF;
    std::vector<double> m_axisReferenceVelocitiesEstimationKF;
    std::vector<double> m_axisReferenceAccelerationEstimationKF;
    Eigen::MatrixXd m_feedbackAxisCovEstimationKF;
    Eigen::MatrixXd m_referenceAxisCovEstimationKF;

    double m_Value_error_threshold_transient;

    bool m_moveRobot; /**< the option to give the user the possibility to do not move the robot
                         (default :: true)*/

    std::unique_ptr<RobotController> m_robotLeftHand; /**< Pointer to the left
                                                              finger retargeting object. */

    std::unique_ptr<HapticGlove::GloveControlHelper> m_gloveLeftHand; /**< Pointer to the left
                                                               hand glove object. */

    std::unique_ptr<HapticGlove::Retargeting> m_retargetingLeftHand;

    double m_calibrationTimePeriod; /**< calibration time period [sec] */

    // Enable at the end
    bool m_enableLogger; /**< log the data (if ON) */
    //    class Logger;
    //    std::unique_ptr<Logger> m_loggerLeftHand;

    /**
     * Get all the feedback signal from the interfaces
     * @return true in case of success and false otherwise.
     */
    bool getFeedbacks();

    bool run();

    bool prepare(bool& isPrepared);

public:
    BilateralTeleoperation();

    ~BilateralTeleoperation();

    /**
     * Configure the wearable implemenetation
     * @param config configuration options
     * @param name name of the robot
     * @param rightHand if true the right hand is used
     * @return true/false in case of success/failure
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close();
};

#endif // BILATERALTELEOPERATION_HPP
