/**
 * @file FingersRetargeting.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <FingersRetargeting.hpp>
#include <Utils.hpp>

bool FingersRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    m_controlHelper = std::make_unique<RobotControlHelper>();
    if (!m_controlHelper->configure(config, name, false))
    {
        yError() << "[FingersRetargeting::configure] Unable to configure the finger helper";
        return false;
    }

    int fingersJoints = m_controlHelper->getDoFs();

    double samplingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[configure] Unable to find the head smoothing time";
        return false;
    }

    m_fingersScaling.resize(fingersJoints);
    if (!YarpHelper::getYarpVectorFromSearchable(config, "fingersScaling", m_fingersScaling))
    {
        yError() << "[configure] Initialization failed while reading "
                    "fingersVelocity vector.";
        return false;
    }

    m_desiredJointPosition.resize(fingersJoints);
    yarp::sig::Vector buff(fingersJoints, 0.0);
    yarp::sig::Matrix limits(fingersJoints, 2);
    if (!m_controlHelper->getLimits(limits))
    {
        yError() << "[FingersRetargeting::configure] Unable to get the joint limits.";
        return false;
    }
    m_fingerIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buff, limits);

    // switch to position direct mode. Notice it might be nice to control the fingers in
    // velocity mode. Indeed now we are setting a desired joint velocity and we evaluate
    // the joint position using an integrator.
    if (!m_controlHelper->switchToControlMode(VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "unable to switch the control mode";
        return false;
    }

    return true;
}

bool FingersRetargeting::setFingersVelocity(const double& fingersVelocity)
{
    if (m_fingerIntegrator == nullptr)
    {
        yError() << "[FingersRetargeting::setFingersVelocity] the integrator is not initialize "
                    "please call configure() method";
        return false;
    }

    m_desiredJointPosition = m_fingerIntegrator->integrate(fingersVelocity * m_fingersScaling);
    return true;
}
