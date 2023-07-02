/**
 * @file OpenXRJoypadModule.hpp
 * @copyright 2023 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2023
 */

#ifndef OPENXRJOYPAD_MODULE_HPP
#define OPENXR_MODULE_HPP

// std
#include <memory>

// YARP
#include <yarp/os/RFModule.h>

class OpenXRJoypadModule : public yarp::os::RFModule
{
private:
    struct Impl;
    std::unique_ptr<Impl> m_pImpl;

public:
    OpenXRJoypadModule();
    ~OpenXRJoypadModule();
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() final;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() final;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) final;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() final;
};

#endif // OPENXR
