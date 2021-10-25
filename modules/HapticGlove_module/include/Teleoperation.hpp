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
#include <RobotController_hapticGlove.hpp>

namespace HapticGlove
{
class Teleoperation;
}

class HapticGlove::Teleoperation
{
    std::string m_logPrefix;

    double m_dT; /**< module period. */

    std::string m_robot; /**< robot name. */

    bool m_getHumanMotionRange; /**< get the motion range of the human joints*/

    // robot data
    std::vector<double> m_robotAxisReference;
    std::vector<double> m_robotAxisFeedback;
    std::vector<double> m_robotAxisVelocityFeedback;

    std::vector<double> m_robotJointReferences;
    std::vector<double> m_robotJointFeedbacks;

    std::vector<double> m_robotAxisValueErrors;
    std::vector<double> m_robotAxisVelocityErrors;

    std::vector<double> m_robotAxisValuesReferenceKF;
    std::vector<double> m_robotAxisVelocitiesReferenceKF;
    std::vector<double> m_robotAxisAccelerationReferenceKF;
    Eigen::MatrixXd m_robotAxisCovReferenceKF;

    std::vector<double> m_robotAxisValuesFeedbackKF;
    std::vector<double> m_robotAxisVelocitiesFeedbackKF;
    std::vector<double> m_robotAxisAccelerationFeedbackKF;
    Eigen::MatrixXd m_robotAxisCovFeedbackKF;

    // human data
    std::vector<double> m_humanJointValues;
    std::vector<double> m_humanVibrotactileFeedbacks;
    std::vector<double> m_humanForceFeedbacks;

    bool m_moveRobot; /**< the option to give the user the possibility to do not move the robot
                         (default :: true)*/

    double m_timeConfigurationEnd;

    std::unique_ptr<RobotController> m_robotController; /**< pointer to the robot controller. */

    std::unique_ptr<GloveControlHelper> m_humanGlove; /**< pointer to the human glove object. */

    std::unique_ptr<Retargeting> m_retargeting; /**< pointer to the human retargeting object. */

    double m_calibrationTimePeriod; /**< calibration time period [sec] */

    // Enable at the end
    bool m_enableLogger; /**< log the data (if ON) */
    class Logger;
    std::unique_ptr<Logger> m_loggerLeftHand;

    /**
     * Get all the feedback signal from the interfaces
     * @return true in case of success and false otherwise.
     */
    bool getFeedbacks();

public:
    Teleoperation();

    ~Teleoperation();

    /**
     * Configure the bilteral teleoperation implementation
     * @param config configuration options
     * @param name name of the robot
     * @param rightHand if true the right hand is used
     * @return true/false in case of success/failure
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Close the class.
     * @return true/false in case of success/failure
     */
    bool close();

    bool run();

    bool prepare(bool& isPrepared);
};

#endif // TELEOPERATION_HPP
