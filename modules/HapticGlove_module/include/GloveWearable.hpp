/**
 * @file GloveWearable.hpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef GLOVE_WEARABLE_HPP
#define GLOVE_WEARABLE_HPP

#include <Wearable/IWear/IWear.h>
#include <thrift/WearableActuatorCommand.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

#include <vector>
#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

using namespace yarp::os;

class GloveWearableImpl
{
private:
    yarp::dev::PolyDriver m_wearableDevice;

    wearable::IWear* m_iWear{nullptr}; /**< Sense glove wearable interface. */

    BufferedPort<wearable::msg::WearableActuatorCommand> m_iWearActuatorPort;

    std::string m_handLinkName;

    std::string m_wearablePrefix;

    std::vector<std::string> m_humanJointNameList;

    std::vector<std::string> m_humanFingerNameList;

    wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor> m_handLinkSensor;

    std::vector<wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>> m_jointSensors;

    std::vector<wearable::SensorPtr<const wearable::actuator::IHaptic>> m_ForceFeedbackActuators;

    std::vector<wearable::SensorPtr<const wearable::actuator::IHaptic>> m_VibroTactileActuators;

public:
    GloveWearableImpl();

    ~GloveWearableImpl();

    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    bool createWearableDataVectors();

    bool getFingersJointValues(std::vector<double>& values);

    bool getPalmImuRotationValues(std::vector<double>& values);

    bool getFingertipsPoseValues(Eigen::MatrixXd& measuredValue);

    bool setFingersForceFeedbackValues(std::vector<double>& values);

    bool setFingersVibroTactileValues(std::vector<double>& values);
};

#endif
