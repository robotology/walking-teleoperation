/**
 * @file HapticGloveFingersRetargeting.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef FINGERS_RETARGETING_HPP
#define FINGERS_RETARGETING_HPP

// std
#include <iostream>
#include <memory>

// Eigen
#include <Eigen/Dense>

// YARP
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

#include <HapticGloveRetargetingController.hpp>

using namespace yarp::math;

/**
 * Class useful to manage the retargeting of fingers.
 */
class FingersRetargeting : public RetargetingController
{
private:
    yarp::sig::Vector m_fingersScaling; /**< It contains the finger velocity scaling. */

    std::unique_ptr<iCub::ctrl::Integrator> m_fingerIntegrator{nullptr}; /**< Velocity integrator */

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_A; /**< Coupling Matrix from the motors to the joints; Dimension <n,m> n: number of
                joints, m: number of motors; we have q= m_A x m where q is the joint values and m is
                the motor values*/

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_controlCoeff; /**< control coeeficient matrix from the joints to the motors; Dimension
                <m,q> q: number of joints, m: number of motors*/

    bool m_doCalibration; /**< check if we need to do calibraition */
    bool m_motorJointsCoupled; /**< check if the motors and joints are coupled */

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_motorsData; /**< The logged data for calibration; the motors values; Dimension <o, m> o:
                         number of observations (logged data), m: number of motors */

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_jointsData; /**< The logged data for calibration; joints values; Dimension <o, n> o:
                         number of observations (logged data), m: number of joints */

    yarp::sig::Vector motorVelocityReference;

public:
    /**
     * Configure the object.
     * @param config reference to a resource finder object.
     * @param name name of the robot
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    /**
     * Set the fingers axis reference value
     * @param fingersReference the reference value for the finger axis to follow
     * @return true in case of success and false otherwise.
     */
    bool setFingersAxisReference(const yarp::sig::Vector& fingersReference);

    /**
     * Set the fingers joint reference value
     * @param fingersReference the reference value for the finger joints to follow
     * @return true in case of success and false otherwise.
     */
    bool setFingersJointReference(const yarp::sig::Vector& fingersReference);

    /**
     * Get the fingers' axis velocities or values
     * @param fingerValue get the fingers' axis velocity or value
     */
    void getFingerAxisMeasuredValues(yarp::sig::Vector& fingerValues);

    /**
     * Get the fingers joint velocities or values
     * @param fingerValue get the finger joint velocity or value
     */
    void getFingerJointsMeasuredValues(yarp::sig::Vector& fingerValues);

    /**
     * Update the feedback values
     */
    bool updateFeedback(void);

    /**
     * Find the coupling relationship bwtween the motors and joitns of the robots
     * @return true if it could open the logger
     */
    bool LogDataToCalibrateRobotMotorsJointsCouplingRandom(const bool generateRandomVelocity);

    /**
     * Find the coupling relationship bwtween the motors and joitns of the robots using sin input
     * function
     * @param time the current time
     * @param axisNumber the axis to control and give input
     * @return true if it could open the logger
     */
    bool LogDataToCalibrateRobotMotorsJointsCouplingSin(double time, int axisNumber);

    /**
     * Solving the linear regression problem to compute the matrix m_A
     * @return true if it could open the logger
     */
    bool trainCouplingMatrix();
};

template <typename DynamicEigenMatrix, typename DynamicEigenVector>
bool push_back_row(DynamicEigenMatrix& m, const DynamicEigenVector& values)
{
    if (m.size() != 0)
    {
        if (m.cols() != values.cols())
        {

            std::cerr << "[push_back_row]: the number of columns in matrix and vactor are not "
                         "equal; m.cols(): "
                      << m.cols() << ", values.cols():" << values.cols() << std::endl;
            return false;
        }
    }
    Eigen::Index row = m.rows();
    m.conservativeResize(row + 1, values.cols());
    for (int i = 0; i < values.cols(); i++)
        m(row, i) = values(0, i);
    return true;
}
#endif
