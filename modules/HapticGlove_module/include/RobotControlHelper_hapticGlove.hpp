/**
 * @file HapticGloveRobotControlHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef ROBOT_CONTROL_HELPER_HAPTIC_GLOVE_HPP
#define ROBOT_CONTROL_HELPER_HAPTIC_GLOVE_HPP

// std
#include <memory>

// YARP
#include <yarp/sig/Vector.h>
// teleoperation
#include <RobotControlInterface_hapticGlove.hpp>
// eigen
#include <Eigen/Dense>

using namespace yarp::math;
// using namespace HapticGlove;

/**
 * RobotControlHelper is a virtual class for helping to control one part of the robot (i.e. head or
 * fingers)
 */
class RobotControlHelper
{
protected:
    std::unique_ptr<HapticGlove::RobotControlInterface>
        m_robotControlInterface; /**< Controller helper */
    std::vector<double> m_axisValueReferences; /** Reference axis value in radiant or radiant/s  */
    std::vector<double> m_axisValueFeedbacks; /** Feedback axis value in radiant or radiant/s  */

    Eigen::VectorXd m_axisValueReferencesEigen; /** Reference axis value in radiant or radiant/s  */
    Eigen::VectorXd m_axisValueFeedbacksEigen; /** Feedback axis value in radiant or radiant/s  */

    std::vector<double>
        m_jointValueReferences; /** Reference joint values in radiant or radiant/s  */

    std::vector<double>
        m_jointValuesExpected; /** Reference joint values in radiant or radiant/s  */

    std::vector<double> m_jointValueFeedbacks; /** Feedback joint values in radiant or radiant/s  */

    Eigen::VectorXd
        m_jointValueReferencesEigen; /** Reference joint values in radiant or radiant/s  */
    Eigen::VectorXd m_jointValuesExpectedEigen;
    Eigen::VectorXd
        m_jointValueFeedbacksEigen; /** Feedback joint values in radiant or radiant/s  */

public:
    /**
     * Configure the object.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    virtual bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand)
        = 0;

    /**
     * Move the robot part
     * @return true in case of success and false otherwise.
     */
    virtual bool move();

    /**
     * Expose the contolHelper interface (const)
     * @return control helper interface
     */
    const std::unique_ptr<HapticGlove::RobotControlInterface>& controlHelper() const;

    /**
     * Expose the contolHelper interface
     * @return control helper interface
     */
    std::unique_ptr<HapticGlove::RobotControlInterface>& controlHelper();
    virtual ~RobotControlHelper();
};
#endif
