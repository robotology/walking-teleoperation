/**
 * @file GloveWearable.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */


#include <GloveWearable.hpp>
#include <GloveControlHelper.hpp>
#include <mutex>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <Utils.hpp>



using namespace wearable;
const std::string LogPrefix="GloveWearableImpl::";

GloveWearableImpl::GloveWearableImpl() { }


GloveWearableImpl::~GloveWearableImpl() { }


bool GloveWearableImpl::configure (const yarp::os::Searchable& config, const std::string& name, const bool& rightHand)
{

    bool m_isRightHand; /**< true if the glove is the right hand*/

    m_isRightHand=rightHand;

    // set Glove Joints List Names From Module
    if (!config.check("hand_link"))
    {
        yError() << "[GloveControlHelper::configure] Unable to find hand_link into config file.";
        return false;
    }
    m_handLinkName=config.find("hand_link").asString();

    // set Glove Joints List Names From Module
    yarp::os::Value* jointListYarp;
    if (!config.check("human_joint_list", jointListYarp))
    {
        yError() << "[GloveControlHelper::configure] Unable to find human_joint_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(jointListYarp, m_humanJointNameList))
    {
        yError() << "[GloveControlHelper::configure] Unable to convert human_joint_list list into a "
                    "vector of strings.";
        return false;
    }

    // set Glove Link Names From Module
    yarp::os::Value* fingerListYarp;
    if (!config.check("human_finger_list", fingerListYarp))
    {
        yError() << "[GloveControlHelper::configure] Unable to find human_finger_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(fingerListYarp, m_humanFingerNameList))
    {
        yError() << "[GloveControlHelper::configure] Unable to convert human_finger_list list into a "
                    "vector of strings.";
        return false;
    }


    yarp::dev::PolyDriver wearableDevice;
    yarp::os::Property options;
    options.put("device", "iwear_remapper");
    yarp::os::Value* wearableDataPort;
    if (!config.check("wearable_data_ports", wearableDataPort))
    {
        yError() << LogPrefix <<"Unable to find wearable_data_ports into config file.";
        return false;
    }
    options.put("wearableDataPorts", wearableDataPort);
    //options.put("useRPC",0);

    yarp::os::Value* wearableRpcPort;
    if (!config.check("wearable_rpc_ports", wearableRpcPort))
    {
        yError() << LogPrefix <<"Unable to find wearable_rpc_ports into config file.";
        return false;
    }

    //    options.put("wearableRPCPorts", wearableRpcPort);

    if(!wearableDevice.open(options))
    {
        yError() << LogPrefix <<"Failed to connect wearable remapper device";
        return false;
    }

    if(!wearableDevice.view(m_iWear) || !m_iWear )
    {
        yError() << LogPrefix <<"Failed to view Glove Wearable Interface";
        return false;
    }

    while (m_iWear->getStatus() == WearStatus::WaitingForFirstRead) {
        yInfo() << LogPrefix <<"(configure)IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
    }

    if (m_iWear->getStatus() != WearStatus::Ok) {
        yError() << LogPrefix <<"(configure) The status of the attached IWear interface is not ok ("
                 << static_cast<int>(m_iWear->getStatus()) << ")";
        return false;
    }

    if(!createWearableDataVectors())
    {
        yError()<<LogPrefix<<"Unbale to create all the necessary Sense Glove Wearable Data maps";
        return false;
    }

    return true;
}


bool GloveWearableImpl::createWearableDataVectors()
{
    std::string wearablePrefix ="HapticGlove::";
    // create link sensors map
    {
        std::string wearableLinkSensorName= wearablePrefix+ wearable::sensor::IVirtualLinkKinSensor::getPrefix()+ m_handLinkName;
        auto sensor = m_iWear->getVirtualLinkKinSensor(wearableLinkSensorName);
        if(!sensor)
        {
            yError() << LogPrefix << "Failed to find sensor associated to link" << wearableLinkSensorName<< "from the IWear interface";
            return false;
        }
        m_linkSensor= sensor;
        yInfo()<<LogPrefix<<"createWearableDataVectors: "<<m_linkSensor->getSensorName();
    }
    // create joints sensors map
    {
        for (auto jointName : m_humanJointNameList)
        {
            std::string wearableJointSensorName= wearablePrefix+ wearable::sensor::IVirtualJointKinSensor::getPrefix()+ jointName;
            auto sensor = m_iWear->getVirtualJointKinSensor(wearableJointSensorName);
            if(!sensor)
            {
                yError() << LogPrefix << "Failed to find sensor associated to joint" << wearableJointSensorName<< "from the IWear interface";
                return false;
            }
            m_jointSensors.push_back(sensor);
            yInfo()<<LogPrefix<<"createWearableDataVectors: "<<sensor->getSensorName();
        }
    }
    yInfo()<<LogPrefix<<"createWearableDataVectors: m_jointSensors.size() "<<m_jointSensors.size();

    // create link force feedback actuator map
    {
        for (auto fingerName : m_humanFingerNameList)
        {
            std::string wearableFingerForceFeedbackActuatorName= wearablePrefix+ wearable::actuator::IHaptic::getPrefix()+ fingerName+ "::ForceFeedback";
            auto actuator = m_iWear->getHapticActuator(wearableFingerForceFeedbackActuatorName);
            if(!actuator)
            {
                yError() << LogPrefix << "Failed to find actuator associated to finger " << wearableFingerForceFeedbackActuatorName<< "from the IWear interface";
                return false;
            }
            m_ForceFeedbackActuators.push_back(actuator);
            yInfo()<<LogPrefix<<"createWearableDataVectors: "<<actuator->getActuatorName();
        }
    }
    yInfo()<<LogPrefix<<"createWearableDataVectors: m_ForceFeedbackActuators.size() "<<m_ForceFeedbackActuators.size();


    // create link vibro tactile feedback actuator map
    {
        for (auto fingerName : m_humanFingerNameList)
        {
            std::string wearableFingerVibroTactileActuatorName= wearablePrefix+ wearable::actuator::IHaptic::getPrefix()+ fingerName+ "::VibroTactileFeedback";
            auto actuator = m_iWear->getHapticActuator(wearableFingerVibroTactileActuatorName);
            if(!actuator)
            {
                yError() << LogPrefix << "Failed to find actuator associated to finger " << wearableFingerVibroTactileActuatorName<< "from the IWear interface";
                return false;
            }
            m_VibroTactileActuators.push_back(actuator);
            yInfo()<<LogPrefix<<"createWearableDataVectors: "<<actuator->getActuatorName();

        }
    }
    yInfo()<<LogPrefix<<"createWearableDataVectors: m_VibroTactileActuators.size() "<<m_VibroTactileActuators.size();

    return false;
}


bool GloveWearableImpl::getSenseGloveHumanJointValues(std::vector<double>& values)
{
    values.clear();
    values.reserve(m_humanJointNameList.size());
    double jointPosition;
    for (auto sensor: m_jointSensors)
    {
        sensor->getJointPosition(jointPosition);
        values.push_back(jointPosition);
    }
    return true;
}


bool GloveWearableImpl::getSenseGloveImuValues(std::vector<double>& values)
{

    values.clear();
    values.reserve(4); // quaternion size
    wearable::Quaternion orientation;
    m_linkSensor->getLinkOrientation(orientation);
    for (size_t i=0; i<4; i++)
        values.push_back(orientation[i]);

    return true;
}

bool GloveWearableImpl::setSenseGloveForceFeedbackValues(std::vector<double>& values)
{
    if (values.size()!=m_ForceFeedbackActuators.size())
    {
        yError()<<LogPrefix<<"The size of force feedback and reference values are different.";
        return false;
    }
    size_t i=0;
    for (auto actuator: m_ForceFeedbackActuators)
    {
        actuator->setHapticCommand(values[i]);
        i++;
    }
    return true;
}

bool GloveWearableImpl::setSenseGloveVibroTactileValues(std::vector<double>& values)
{
    if (values.size()!=m_VibroTactileActuators.size())
    {
        yError()<<LogPrefix<<"The size of vibro tactile feedback and reference values are different.";
        return false;
    }
    size_t i=0;
    for (auto actuator: m_VibroTactileActuators)
    {
        actuator->setHapticCommand(values[i]);
        i++;
    }
    return true;
}


