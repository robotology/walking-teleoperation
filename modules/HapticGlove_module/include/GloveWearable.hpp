// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef GLOVE_WEARABLE_HPP
#define GLOVE_WEARABLE_HPP

// std
#include <Eigen/Dense>
#include <vector>
// wearable
#include <Wearable/IWear/IWear.h>
#include <thrift/WearableActuatorCommand.h>
// YARP
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Searchable.h>

template <typename E> constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

using namespace yarp::os;
namespace HapticGlove
{
class GloveWearableImpl;
namespace SenseGlove
{
enum class ThumperCmd : unsigned int;
} // namespace SenseGlove
} // namespace HapticGlove

enum class HapticGlove::SenseGlove::ThumperCmd : unsigned int
{
    None = 126,

    /// <summary> Turn off the thumper effects. </summary>
    TurnOff = 124,

    /// <summary> A 5-second long, constant vibration. </summary>
    Cue_Game_Over = 118,

    /// <summary> A double-click at 100% intensity. </summary>
    Button_Double_100 = 10,
    /// <summary> A double click at 60% intensity. </summary>
    Button_Double_60 = 11,

    /// <summary> Simulates an impact of the hand at 100% intensity. </summary>
    Impact_Thump_100 = 1,
    /// <summary> Simulates an impact of the hand at 30% intensity. </summary>
    Impact_Thump_30 = 3,
    /// <summary> Simulates an sharp impact of the hand at 40% intensity.
    /// </summary>
    Impact_Thump_10 = 6,

    /// <summary> A light vibration to cue the user that an object it picked up.
    /// 100% intensity. </summary>
    Object_Grasp_100 = 7,
    /// <summary> A light vibration to cue the user that an object it picked up.
    /// 60% intensity. </summary>
    Object_Grasp_60 = 8,
    /// <summary> A light vibration to cue the user that an object it picked up.
    /// 30% intensity. </summary>
    Object_Grasp_30 = 9
};

/**
 * GloveWearableImpl is a class for interfacing with the haptic glove using the wearable interface.
 */
class HapticGlove::GloveWearableImpl
{
private:
    std::string m_logPrefix;

    const size_t m_numForceFeedback; /**< Number of the motors to produce force feedback to the
                         human*/
    const size_t m_numVibrotactileFeedback; /**< Number of the vibrotactile to produce vibrotactile
                                      feedback to the human*/
    const size_t m_numFingers; /**< Number of the fingers of the glove/human */
    const size_t m_numHandJoints; /**< Number of the joints of the human hand*/

    yarp::dev::PolyDriver m_wearableDevice;

    wearable::IWear* m_iWear{nullptr}; /**< Sense glove wearable interface. */

    BufferedPort<wearable::msg::WearableActuatorCommand> m_iWearActuatorPort;

    std::string m_handLinkName;

    std::string m_wearablePrefix;

    std::vector<std::string> m_humanJointNameList;

    std::vector<std::string> m_humanFingerNameList;

    wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor> m_handPalmSensor;

    std::vector<wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>> m_jointSensors;

    std::vector<wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>>
        m_fingertipLinkSensors;

    std::vector<wearable::SensorPtr<const wearable::actuator::IHaptic>>
        m_fingertipForceFeedbackActuators;

    std::vector<wearable::SensorPtr<const wearable::actuator::IHaptic>>
        m_fingertipVibrotactileActuators;

    wearable::SensorPtr<const wearable::actuator::IHaptic> m_palmVibrotactileActuator;

public:
    /**
     * ConstructorbrotactileValues(const std::vector<int>& values);

     * @param numFingers number of fingers
     * @param numForceFeedback number of force feedback actuators on the hand fingertips
     * @param numVibrotactileFeedback number of vibrotactile feedback actuators on the hand
     * fingertips
     * @param numHandJoints number of hand joints
     */
    GloveWearableImpl(const size_t& numFingers,
                      const size_t& numForceFeedback,
                      const size_t& numVibrotactileFeedback,
                      const size_t& numHandJoints);

    /**
     * Destructor
     */
    ~GloveWearableImpl();

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
     * initialize the Wearable data vectors associated with the sensors
     * @return true/false in case of success/failure
     */
    bool initializeWearableSensors();

    /**
     * get the sensor data associated with the human hand joints
     * @param values the vector of human joint angles [rad]
     * @return true/false in case of success/failure
     */
    bool getJointValues(std::vector<double>& values);

    /**
     * get the sensor data associated with the human hand palm rotation
     * @param values the vector of human hand palm quaternion [w, x, y, z]
     * @return true/false in case of success/failure
     */
    bool getPalmImuRotationValues(std::vector<double>& values);

    /**
     * get the sensor data associated with all the human hand fingertip poses [position(x, y, z),
     * quaternion(w, x, y, z)]
     * @param values an eigen matrix providing all the human hand fingertip poses (number of
     * fingertips x pose size (7))
     * -- rows: from thumb to pinky finger
     * -- columns:[position(x, y, z), quaternion(w, x, y, z)]
     * @return true/false in case of success/failure
     */
    bool getFingertipPoseValues(Eigen::MatrixXd& values);

    /**
     * set the force feedback actuator values associated with all the human hand fingertips
     * @param values the vector of force feedback values to the human fingertips, from thumb to
     * pinky, range: [0, 100]
     * @return true/false in case of success/failure
     */
    bool setFingertipForceFeedbackValues(const std::vector<int>& values);

    /**
     * set the vibrotactile feedback actuator values associated with all the human hand fingertips
     * @param values the vector of vibrotactile feedback values to the human fingertips, from thumb
     * to pinky, range: [0, 100]
     * @return true/false in case of success/failure
     */
    bool setFingertipVibrotactileValues(const std::vector<int>& values);

    /**
     * set the vibrotactile feedback actuator value associated with the human hand palm
     * @param value the vibrotactile feedback value associated with the human hand palm
     * @return true/false in case of success/failure
     */
    bool setPalmVibrotactileValue(const int& value);

    /**
     * Close the Glove wearable implementation
     * @return true/false in case of success/failure
     */
    bool close();
};

#endif // GLOVE_WEARABLE_HPP
