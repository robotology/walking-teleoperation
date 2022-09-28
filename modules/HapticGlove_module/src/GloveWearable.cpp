/**
 * @file GloveWearable.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <GloveWearable.hpp>
#include <Utils.hpp>
#include <mutex>
#include <yarp/os/LogStream.h>

using namespace wearable;
using namespace HapticGlove;

GloveWearableImpl::GloveWearableImpl(const size_t& numFingers,
                                     const size_t& numForceFeedback,
                                     const size_t& numVibrotactileFeedback,
                                     const size_t& numHandJoints)
    : m_numFingers(numFingers)
    , m_numForceFeedback(numForceFeedback)
    , m_numVibrotactileFeedback(numVibrotactileFeedback)
    , m_numHandJoints(numHandJoints)
{
    m_logPrefix = "GloveWearableImpl::";
}

GloveWearableImpl::~GloveWearableImpl() = default;

bool GloveWearableImpl::configure(const yarp::os::Searchable& config,
                                  const std::string& name,
                                  const bool& rightHand)
{

    m_logPrefix += rightHand ? "RightHand:: " : "LeftHand:: ";

    m_wearablePrefix = "HapticGlove::";

    // set Glove Joints List Names From Module
    if (!config.check("hand_link"))
    {
        yError() << m_logPrefix
                 << "unable to find hand_link into "
                    "config file.";
        return false;
    }
    m_handLinkName = config.find("hand_link").asString();

    // set Glove Joints List Names From Module
    yarp::os::Value* jointListYarp;
    if (!config.check("human_joint_list", jointListYarp))
    {
        yError() << m_logPrefix
                 << "unable to find "
                    "human_joint_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(jointListYarp, m_humanJointNameList))
    {
        yError() << m_logPrefix
                 << "unable to convert human_joint_list list into a vector of strings.";
        return false;
    }

    if (m_humanJointNameList.size() != m_numHandJoints)
    {
        yError() << m_logPrefix
                 << "number of joints in the config file is different from the default value.";
        return false;
    }

    // set Glove Link Names From Module
    yarp::os::Value* fingerListYarp;
    if (!config.check("human_finger_list", fingerListYarp))
    {
        yError() << m_logPrefix << "unable to find human_finger_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(fingerListYarp, m_humanFingerNameList))
    {
        yError() << m_logPrefix
                 << "unable to convert human_finger_list list into a vector of strings.";
        return false;
    }

    if (m_humanFingerNameList.size() != m_numFingers)
    {
        yError() << m_logPrefix
                 << "number of fingers in the config file is different from the default value.";
        return false;
    }

    yarp::os::Property options;
    options.put("device", "iwear_remapper");
    yarp::os::Value* wearableDataPort;
    if (!config.check("wearable_data_ports", wearableDataPort))
    {
        yError() << m_logPrefix << "unable to find wearable_data_ports into config file.";
        return false;
    }
    options.put("wearableDataPorts", wearableDataPort);
    options.put("carrier", "fast_tcp");

    if (!m_wearableDevice.open(options))
    {
        yError() << m_logPrefix << "failed to connect wearable remapper device";
        return false;
    }

    if (!m_wearableDevice.view(m_iWear) || !m_iWear)
    {
        yError() << m_logPrefix << "failed to view Glove Wearable Interface";
        return false;
    }

    while (m_iWear->getStatus() == WearStatus::WaitingForFirstRead)
    {
        yInfo() << m_logPrefix << "IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(0.1);
    }

    if (m_iWear->getStatus() != WearStatus::Ok)
    {
        yError() << m_logPrefix << "the status of the attached IWear interface is not ok ("
                 << static_cast<int>(m_iWear->getStatus()) << ")";
        return false;
    }
    yInfo() << "Attached wearable device name : " << m_iWear->getWearableName();

    std::string portNameOut, portNameIn;
    if (!YarpHelper::getStringFromSearchable(
            config, "wearable_data_actuator_ports_out", portNameOut))
    {
        yError() << m_logPrefix
                 << "Unable to get a string from a searchable: "
                    "wearable_data_actuator_ports_out";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(config, "wearable_data_actuator_ports_in", portNameIn))
    {
        yError() << m_logPrefix
                 << "unable to get a string from a searchable: "
                    "wearable_data_actuator_ports_in";
        return false;
    }

    if (!m_iWearActuatorPort.open(portNameOut))
    {
        yError() << m_logPrefix << portNameOut << " port already open.";
        return false;
    }

    if (!Network::connect(m_iWearActuatorPort.getName(), portNameIn, "fast_tcp"))
    {
        yError() << m_logPrefix << "output port: " << portNameOut << "input port: " << portNameIn
                 << " unable to connect the ports.";
        return false;
    }

    if (!initializeWearableSensors())
    {
        yError() << m_logPrefix
                 << "unbale to create all the necessary Sense Glove Wearable Data maps ";
        return false;
    }

    yInfo() << m_logPrefix << "configuration is done.";
    return true;
}

bool GloveWearableImpl::initializeWearableSensors()
{

    // intialize palm (or hand) link sensor
    {
        std::string wearableLinkSensorName = m_wearablePrefix
                                             + wearable::sensor::IVirtualLinkKinSensor::getPrefix()
                                             + m_handLinkName;
        auto sensor = m_iWear->getVirtualLinkKinSensor(wearableLinkSensorName);
        if (!sensor)
        {
            yError() << m_logPrefix << "failed to find sensor associated to link"
                     << wearableLinkSensorName << "from the IWear interface";
            return false;
        }
        m_handPalmSensor = sensor;
        yInfo() << m_logPrefix
                << "initialize the following sensor: " << m_handPalmSensor->getSensorName();
    }

    // intialize fingertip link sensors
    {
        m_fingertipLinkSensors.reserve(m_numFingers);
        for (const auto& fingerName : m_humanFingerNameList)
        {
            std::string wearableSensorName = m_wearablePrefix
                                             + wearable::sensor::IVirtualLinkKinSensor::getPrefix()
                                             + fingerName + "::fingertip";
            auto sensor = m_iWear->getVirtualLinkKinSensor(wearableSensorName);
            if (!sensor)
            {
                yError() << m_logPrefix << "failed to find sensor associated to link"
                         << wearableSensorName << "from the IWear interface";
                return false;
            }
            m_fingertipLinkSensors.push_back(sensor);
            yInfo() << m_logPrefix
                    << "initialize the following sensor: " << sensor->getSensorName();
        }

        yInfo() << m_logPrefix
                << "size of the fingertip link sensors: " << m_fingertipLinkSensors.size();
    }

    // intialize joints sensors
    {
        m_jointSensors.reserve(m_numHandJoints);
        for (const auto& jointName : m_humanJointNameList)
        {
            std::string wearableSensorName = m_wearablePrefix
                                             + wearable::sensor::IVirtualJointKinSensor::getPrefix()
                                             + jointName;
            auto sensor = m_iWear->getVirtualJointKinSensor(wearableSensorName);
            if (!sensor)
            {
                yError() << m_logPrefix << "failed to find sensor associated to joint"
                         << wearableSensorName << "from the IWear interface";
                return false;
            }
            m_jointSensors.push_back(sensor);
            yInfo() << m_logPrefix
                    << "initialize the following sensor: " << sensor->getSensorName();
        }
        yInfo() << m_logPrefix << "size of the joint sensors: " << m_jointSensors.size();
    }
    yInfo() << m_logPrefix << "initialization of the wearable sensors is done.";

    return true;
}

bool GloveWearableImpl::getJointValues(std::vector<double>& values)
{
    if (values.size() != m_numHandJoints)
        values.resize(m_numHandJoints, 0.0);

    size_t i = 0;
    for (const auto& sensor : m_jointSensors)
    {
        if (sensor->getSensorStatus() != wearable::sensor::SensorStatus::Ok)
        {
            std::string sName = sensor->getSensorName();
            yError() << m_logPrefix << "sensor status is not OK, sensor name: " << sName;
        }
        sensor->getJointPosition(values[i]);
        i++;
    }
    return true;
}

bool GloveWearableImpl::getPalmImuRotationValues(std::vector<double>& values)
{
    if (values.size() != 4) // quaternion size
        values.resize(4, 0.0);

    if (m_handPalmSensor->getSensorStatus() != wearable::sensor::SensorStatus::Ok)
    {
        std::string sName = m_handPalmSensor->getSensorName();
        yError() << m_logPrefix << "sensor status is not OK, sensor name: " << sName;
    }

    wearable::Quaternion orientation;
    m_handPalmSensor->getLinkOrientation(orientation);
    for (size_t i = 0; i < 4; i++)
        values[i] = orientation[i];
    return true;
}

bool GloveWearableImpl::getFingertipPoseValues(Eigen::MatrixXd& values)
{
    if (values.rows() != m_numFingers && values.cols() != 7)
        values.resize(m_numFingers, 7);

    size_t i = 0;
    for (const auto& sensor : m_fingertipLinkSensors)
    {
        if (sensor->getSensorStatus() != wearable::sensor::SensorStatus::Ok)
        {
            std::string sName = sensor->getSensorName();
            yError() << m_logPrefix << "sensor status is not OK, sensor name: " << sName;

            wearable::sensor::SensorStatus status = sensor->getSensorStatus();
            switch (status)
            {
            case wearable::sensor::SensorStatus::Ok:
                yInfo() << "sensor status: ok";
                break;
            case wearable::sensor::SensorStatus::Error:
                yInfo() << "sensor status: Error";
                break;
            case wearable::sensor::SensorStatus::Calibrating:
                yInfo() << "sensor status: Calibrating";
                break;
            case wearable::sensor::SensorStatus::Overflow:
                yInfo() << "sensor status: Overflow";
                break;
            case wearable::sensor::SensorStatus::Timeout:
                yInfo() << "sensor status: Timeout";
                break;
            case wearable::sensor::SensorStatus::Unknown:
                yInfo() << "sensor status: Unknown";
                break;
            case wearable::sensor::SensorStatus::WaitingForFirstRead:
                yInfo() << "sensor status: WaitingForFirstRead";
                break;
            default:
                yInfo() << "default case!";
                break;
            }
        }

        wearable::Quaternion orientation;
        wearable::Vector3 position;
        sensor->getLinkPose(position, orientation);
        for (size_t j = 0; j < 3; j++)
            values(i, j) = position[j];
        for (size_t j = 0; j < 4; j++)
            values(i, j + 3) = orientation[j];
        i++;
    }
    return true;
}

bool GloveWearableImpl::setFingertipForceFeedbackValues(const std::vector<int>& values)
{
    if (values.size() != m_numForceFeedback)
    {
        yError() << m_logPrefix
                 << "size of the force feedback vector is not equal to the size of the default "
                    "force feedback size.";
    }
    for (size_t i = 0; i < values.size(); i++)
    {
        std::string fingerName = m_humanFingerNameList[i];

        wearable::msg::WearableActuatorCommand& wearableActuatorCommand
            = m_iWearActuatorPort.prepare();

        wearableActuatorCommand.value = values[i];
        wearableActuatorCommand.info.name = m_wearablePrefix
                                            + wearable::actuator::IHaptic::getPrefix() + fingerName
                                            + "::ForceFeedback";
        wearableActuatorCommand.info.type = wearable::msg::ActuatorType::HAPTIC;
        wearableActuatorCommand.duration = 0;

        m_iWearActuatorPort.write(true); // writeStrict option for wearable haptic device should be
                                         // set to true to avoid the data loss for all actuators
    }
    return true;
}

bool GloveWearableImpl::setFingertipVibrotactileValues(const std::vector<int>& values)
{
    if (values.size() != m_numVibrotactileFeedback)
    {
        yError()
            << m_logPrefix
            << "size of the vibrotactile feedback vector is not equal to the size of the default "
               "vibrotactile feedback size.";
    }

    for (size_t i = 0; i < values.size(); i++)
    {
        std::string fingerName = m_humanFingerNameList[i];

        wearable::msg::WearableActuatorCommand& wearableActuatorCommand
            = m_iWearActuatorPort.prepare();

        wearableActuatorCommand.value = values[i];
        wearableActuatorCommand.info.name = m_wearablePrefix
                                            + wearable::actuator::IHaptic::getPrefix() + fingerName
                                            + "::VibroTactileFeedback";

        wearableActuatorCommand.info.type = wearable::msg::ActuatorType::HAPTIC;
        wearableActuatorCommand.duration = 0;

        m_iWearActuatorPort.write(true); // writeStrict option for wearable haptic device should be
                                         // set to true to avoid the data loss for all actuators
    }

    return true;
}

bool GloveWearableImpl::setPalmVibrotactileValue(const int& value)
{
    /** Helper: check the Wearable device for more info
     * None = 126,
     * TurnOff = 124,
     * Cue_Game_Over = 118,
     * Button_Double_100 = 10,
     * Button_Double_60 = 11,
     * Impact_Thump_100 = 1,
     * Impact_Thump_30 = 3,
     * Impact_Thump_10 = 6,
     * Object_Grasp_100 = 7,
     * Object_Grasp_60 = 8,
     * Object_Grasp_30 = 9
     */

    wearable::msg::WearableActuatorCommand& wearableActuatorCommand = m_iWearActuatorPort.prepare();

    wearableActuatorCommand.value = value;
    wearableActuatorCommand.info.name = m_wearablePrefix + wearable::actuator::IHaptic::getPrefix()
                                        + m_handLinkName + "::VibroTactileFeedback";
    wearableActuatorCommand.info.type = wearable::msg::ActuatorType::HAPTIC;
    wearableActuatorCommand.duration = 0;

    m_iWearActuatorPort.write(false);
    return true;
}

bool GloveWearableImpl::close()
{
    yInfo() << m_logPrefix << "closing the glove wearable implementation.";
    m_iWear = nullptr;
    m_wearableDevice.close();

    m_iWearActuatorPort.close();

    return true;
}
