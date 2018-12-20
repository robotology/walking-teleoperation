/**
 * @file RetargetingHelper.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef RETARGETING_HELPER_HPP
#define RETARGETING_HELPER_HPP

// std
#include <memory>

// YARP
#include <yarp/sig/Vector.h>

#include <RobotControlHelper.hpp>

using namespace yarp::math;

/**
 * Class useful to manage the retargeting
 */
class RetargetingHelper
{
protected:
    std::unique_ptr<RobotControlHelper> m_controlHelper;
    yarp::sig::Vector m_desiredJointPosition;

public:
    /**
     * Configure the object.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    virtual bool configure(const yarp::os::Searchable& config, const std::string& name) = 0;

    /**
     * Move the part
     * @return true in case of success and false otherwise.
     */
    bool move();

    /**
     * Expose the contolHelper interface (const)
     * @return control helper interface
     */
    const std::unique_ptr<RobotControlHelper>& controlHelper() const;

    /**
     * Expose the contolHelper interface (const)
     * @return control helper interface
     */
    std::unique_ptr<RobotControlHelper>& controlHelper();
};
#endif
