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
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>



namespace wearable {
    namespace devices {
        class Glove;
    } // namespace devices
} // namespace wearable

class wearable::devices::Glove :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IWrapper,
        public wearable::IWear
{
private:
    class SenseGloveImpl;
    std::unique_ptr<SenseGloveImpl> pImpl;
public:
    Glove();
    ~Glove() override;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IWear
    WearableName getWearableName() const override;
    WearStatus getStatus() const override;

    inline SensorPtr<const sensor::IOrientationSensor>
    getOrientationSensor(const sensor::SensorName /*name*/) const override;

    bool configure (const yarp::os::Searchable& config, const std::string& name);

    bool update(const std::vector<double>& imuData);

};


#endif
