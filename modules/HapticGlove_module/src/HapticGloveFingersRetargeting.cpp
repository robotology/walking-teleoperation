/**
 * @file HapticGloveFingersRetargeting.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <HapticGloveFingersRetargeting.hpp>
#include <Utils.hpp>

bool FingersRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    m_controlHelper = std::make_unique<HapticGlove::RobotControlHelper>();
    if (!m_controlHelper->configure(config, name, false))
    {
        yError() << "[FingersRetargeting::configure] Unable to configure the control helper";
        return false;
    }

    size_t fingersNoOfAxis = m_controlHelper->getActuatedDoFs();

    double samplingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[FingersRetargeting::configure] Unable to find the sampling time";
        return false;
    }

    m_fingersScaling.resize(fingersNoOfAxis);
    if (!YarpHelper::getYarpVectorFromSearchable(config, "fingersScaling", m_fingersScaling))
    {
        yError() << "[FingersRetargeting::configure] Initialization failed while reading "
                    "fingersScaling vector.";
        return false;
    }

    m_desiredJointValue.resize(fingersNoOfAxis);
    yarp::sig::Vector buff(fingersNoOfAxis, 0.0);
    yarp::sig::Matrix limits(fingersNoOfAxis, 2);
    if (!m_controlHelper->getLimits(limits))
    {
        yError() << "[FingersRetargeting::configure] Unable to get the joint limits.";
        return false;
    }
    m_fingerIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buff, limits);

    return true;
}

bool FingersRetargeting::setFingersAxisReference(const yarp::sig::Vector& fingersReference)
{

    if (fingersReference.size() != m_desiredJointValue.size())
    {
        yError() << "[FingersRetargeting::setFingersAxisReference] the size of the "
                    "fingersReference and m_desiredJointValue does not match.";
        return false;
    }

    for (unsigned i = 0; i < fingersReference.size(); i++)
    {
        m_desiredJointValue(i) = fingersReference(i) * m_fingersScaling(i);
    }

    //    if (m_fingerIntegrator == nullptr)
    //    {
    //        yError() << "[FingersRetargeting::setFingersVelocity] The integrator is not initialize
    //        "
    //                    "please call configure() method";
    //        return false;
    //    }
    //    // TOCHECK: maybe remove m_fingersScaling
    //    if (m_controlHelper->isVelocityControlUsed())
    //        m_desiredJointValue = fingersReference * m_fingersScaling; // velocity reference value
    //    else
    //        m_desiredJointValue = m_fingerIntegrator->integrate(fingersReference *
    //        m_fingersScaling);

    return true;
}
bool FingersRetargeting::updateFeedback()
{
    if (!controlHelper()->getFeedback())
    {
        yInfo()
            << "[FingersRetargeting::getFingerAxisValues] Unable the get the finger axis values "
               "from the robot.";
        return false;
    }
    return true;
}

void FingersRetargeting::getFingerAxisMeasuredValues(yarp::sig::Vector& fingerAxisValues)
{
    fingerAxisValues.clear();
    fingerAxisValues = controlHelper()->jointEncoders();
}

void FingersRetargeting::getFingerJointsMeasuredValues(yarp::sig::Vector& fingerJointsValues)
{
    fingerJointsValues.clear();
    fingerJointsValues = controlHelper()->analogSensors();
}
