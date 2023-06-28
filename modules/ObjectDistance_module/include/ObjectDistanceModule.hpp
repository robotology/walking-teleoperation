/**
 * @file module.h
 * @authors
 * @copyright
 */

#ifndef WALKING_TELEOPERATION_OBJECT_DISTANCE_MODULE_H
#define WALKING_TELEOPERATION_OBJECT_DISTANCE_MODULE_H

// std
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// YARP
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>

class ObjectDistanceModule : public yarp::os::RFModule
{
    yarp::dev::PolyDriver m_cameraDevice;
    BipedalLocomotion::RobotInterface::YarpCameraBridge m_cameraBridge;
    yarp::os::BufferedPort<yarp::os::Bottle> m_outputPort;

public:
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;
};

#endif // WALKING_TELEOPERATION_OBJECT_DISTANCE_MODULE_H