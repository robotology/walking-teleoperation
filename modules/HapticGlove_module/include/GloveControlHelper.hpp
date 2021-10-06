/**
 * @file GloveControlHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef GLOVE_CONTROL_HELPER_HPP
#define GLOVE_CONTROL_HELPER_HPP

#include <Eigen/Dense>

// std
#include <memory>

// YARP

#include <yarp/sig/Vector.h>

// Sense Glove
// include "DeviceList.h"
#include "GloveWearable.hpp"
//#include "SenseGlove.h"
/**
 * GloveControlHelper is an helper class for controlling the glove.
 */
namespace HapticGlove
{

class GloveControlHelper
{

    int m_forceFbDof; /**< Number of the actuated motors Dofs to produce force feedback to the
                         human*/
    int m_buzzDof; /**< Number of the actuated vibro tactile Dofs to produce vibro tactile
                         feedback to the human*/
    int m_handNoLinks; /**< Number of the links of the hand model*/

    int m_gloveNoLinks; /**< Number of the links of the hand model*/

    int m_NoSensors; /**< Number of the sensors of the glove */

    bool m_isReady; /**< true if the glove is ready to use, communication working*/

    bool m_isRightHand; /**< true if the glove is the right hand*/

    std::vector<double> m_desiredForceValues; /**< Desired joint value [deg or deg/s]. */
    std::vector<double> m_desiredBuzzValues; /**< Joint position [deg]. */
    std::vector<float> m_sensorData; /**< sensory data of the glove in degree */
    Eigen::MatrixXd m_glovePose; /**< sensory data of the glove poses*/
    Eigen::MatrixXd m_handPose; /**< sensory data of the hand link poses;  From thumb to pinky,
                                proximal to distal; pos [x y z] Quat [x y z w]*/

    Eigen::MatrixXd m_handJointsAngles; /**< sensory data of the hand joints angles;  From thumb to
                                pinky, proximal to distal [rad] [Pronation/Supination (x),
                                Flexion/Extension (y), Abduction/Adduction (z)]*/

    yarp::sig::Vector m_jointsFeedbackInRadians; /**< Joint position [rad]. */

    std::vector<std::string> m_humanJointNameList;
    std::vector<std::string> m_humanFingerNameList;

    std::vector<double> m_jointRangeMin;
    std::vector<double> m_jointRangeMax;

    //    SGCore::SG::SenseGlove m_glove;

    std::unique_ptr<GloveWearableImpl>
        m_pImp; /**< Sense glove wearable interface impelemntation. */

public:
    /**
     * Configure the helper
     * @param config confifuration options
     * @param name name of the robot
     * @param isMandatory if true the helper will return an error if there is a
     * problem in the configuration phase
     * @return true / false in case of success / failure
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Get the measured joints force
     * @param measuredValue measured joint force
     * @return true / false in case of success / failure
     */
    bool getFingersForceMeasured(yarp::sig::Vector& measuredValue);

    /**
     * Get the measured joint values
     * @param measuredValue measured joint values
     * @return true / false in case of success / failure
     */
    bool getHandPose(Eigen::MatrixXd& measuredValue);

    bool getHandJointsAngles();

    bool getHandJointsAngles(std::vector<double>& jointAngleList);

    bool getHandJointsAngles(Eigen::MatrixXd measuredValue);

    bool getGlovePose(Eigen::MatrixXd& measuredValue);

    bool getSensorData(std::vector<float>& measuredValues);

    /**
     * Set the desired joint reference (position or velocity)
     * @param desiredValue desired joint velocity or position (radiant or radiant/s)
     * @return true / false in case of success / failure
     */
    bool setFingersForceReference(const yarp::sig::Vector& desiredValue);

    /**
     * Set the number of vibro-tactile reference
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool setBuzzMotorsReference(const yarp::sig::Vector& desiredValue);

    bool setPalmFeedbackThumper(const int desiredValue);

    /**
     * Set the number of vibro-tactile reference
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool turnOffBuzzMotors();

    bool turnForceFeedback();

    const int getNoOfBuzzMotors() const;

    int getNoOfForceFeedback();

    /**
     * Setup the communication with the glove
     * @return true / false in case of success / failure
     */
    bool setupGlove();

    bool stopHapticFeedback();

    bool isConnected();

    int getNoHandLinks();
    int getNoGloveLinks();

    int getNoSensors();

    void getHumanJointsList(std::vector<std::string>& jointList) const;

    void getHumanFingersList(std::vector<std::string>& fingerList) const;

    bool findHumanMotionRange();

    void getHumanMotionRange(std::vector<double>& jointRangeMin,
                             std::vector<double>& jointRangeMax);

    /**
     * Get the glove IMU data
     * @param gloveImuData glove IMU data with the order x y z w
     * @return true / false in case of success / failure
     */
    bool getGloveIMUData(std::vector<double>& gloveImuData);

    bool updateGloveWearableData();

    /**
     * Close the helper
     */
    void close();
};
} // namespace HapticGlove

#endif
