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


using namespace wearable;
using namespace wearable::devices;

const std::string DeviceName = "HapticGlove";
const std::string LogPrefix = DeviceName + wearable::Separator;

struct SenseGloveData
{
    double x;
    double y;
    double z;
    double w;
};


class Glove::SenseGloveImpl
{
public:
    mutable std::mutex mutex;
    SenseGloveData gloveData;

    WearableName wearableName;
    std::string portsPrefix;

    // Number of sensors
    const int nSensors = 1;

    // Numbe of actuators
    const int nActuators = 0;

    // First data flag
    bool firstDataRead;

    // 3D Force Sensor
    std::string virtualLinkSensorPrefix;
    const std::string virtualLinkSensorName = "linkInfo";
    class SenseGloveVirtualLinkKinSensor;
    SensorPtr<SenseGloveVirtualLinkKinSensor> senseGloveVirtualLinkKinSensor;

    //***
    SenseGloveImpl();
    bool Update(const std::vector<double>& imuData);

};

Glove::SenseGloveImpl::SenseGloveImpl()
{
    wearableName="SenseGlove";
    portsPrefix ="/wearable/SenseGlove";
}

bool Glove::SenseGloveImpl::Update(const std::vector<double>& imuData)
{
    gloveData.w=imuData[0];
    gloveData.x=imuData[1];
    gloveData.y=imuData[2];
    gloveData.z=imuData[3];
    return true;
}


Glove::Glove(): pImpl(new SenseGloveImpl())
{}


Glove::~Glove() =default;


WearableName Glove::getWearableName() const
{
    return pImpl->wearableName+ wearable::Separator;

}

WearStatus Glove::getStatus() const
{
    return WearStatus::Ok;
}


inline SensorPtr<const sensor::IOrientationSensor>
Glove::getOrientationSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

bool Glove::open(yarp::os::Searchable& config)
{

    return true;
}

bool Glove::close()
{
    return false;
}

bool Glove::configure (const yarp::os::Searchable& config, const std::string& name)
{

    auto virtualLinkSensPrefix = getWearableName() + sensor::IVirtualLinkKinSensor::getPrefix();

    pImpl->virtualLinkSensorPrefix =getWearableName() +sensor::IVirtualLinkKinSensor::getPrefix();
    pImpl->senseGloveVirtualLinkKinSensor= SensorPtr<SenseGloveImpl::SenseGloveVirtualLinkKinSensor>{std::make_shared<SenseGloveImpl::SenseGloveVirtualLinkKinSensor>(pImpl.get(),
                                                                                                                                                       pImpl->virtualLinkSensorPrefix + pImpl->virtualLinkSensorName) };
    return true;
}



bool Glove::attach(yarp::dev::PolyDriver* poly)
{

}
bool Glove::detach()
{

}


bool Glove::update(const std::vector<double>& imuData)
{

    pImpl->Update(imuData);
    return true;
}


// ****************** //
// ****************** //

class Glove::SenseGloveImpl::SenseGloveVirtualLinkKinSensor : public wearable::sensor::IVirtualLinkKinSensor
{
public:
    SenseGloveVirtualLinkKinSensor( Glove::SenseGloveImpl* gloveImplPtr,
                                    const sensor::SensorName& name = {},
                                    const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
                                  : IVirtualLinkKinSensor(name, status),
                                    m_gloveImpl(gloveImplPtr)
    { }

    ~SenseGloveVirtualLinkKinSensor() override = default;

    bool getLinkAcceleration(Vector3& linear, Vector3& angular) const override
    {
        // we do not handle linear and angular accelerations in the current implementation
        linear.fill(0.0);
        angular.fill(0.0);

        return true;
    }

    bool getLinkPose(Vector3& position, Quaternion& orientation) const override
    {
        // we do not handle position in the current implementation
        position.fill(0.0);

        std::vector<double> gloveImuData; // w, x, y, z
        assert(m_gloveImpl != nullptr);

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        orientation = {m_gloveImpl->gloveData.w, m_gloveImpl->gloveData.x, m_gloveImpl->gloveData.y, m_gloveImpl->gloveData.z};

        return true;
    }

    bool getLinkVelocity(Vector3& linear, Vector3& angular) const override
    {
        // we do not handle linear and angular velocity
        angular.fill(0.0);
        linear.fill(0.0);
        return true;
    }

    inline void setStatus(const wearable::sensor::SensorStatus& status)
    {
        m_status = status;
    }

private:
    Glove::SenseGloveImpl* m_gloveImpl{nullptr};
};
