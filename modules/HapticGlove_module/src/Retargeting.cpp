#include <Retargeting.hpp>
#include <Utils.hpp>

Retargeting::    Retargeting(const RobotController & robot, const HapticGlove::GloveControlHelper& human): m_robotHand(&robot),
m_gloveHand(&human)
{



}

bool Retargeting::configure(const yarp::os::Searchable& config, const std::string& name){



    yarp::sig::Vector m_totalGain;
    yarp::sig::Vector m_velocityGain;
    yarp::sig::Vector m_buzzMotorsGain;

    m_totalGain.resize(m_robotHand->controlHelper()->getActuatedDoFs(), 0.0);
    m_velocityGain.resize(m_robotHand->controlHelper()->getActuatedDoFs(), 0.0);
//    m_buzzMotorsGain.resize(m_gloveHand->getNoOfBuzzMotors(), 0.0);

    if(!YarpHelper::getYarpVectorFromSearchable(config, "K_GainTotal", m_totalGain))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading K_GainTotal vector of the hand.";
        return false;
    }
    if(!YarpHelper::getYarpVectorFromSearchable(config, "K_GainVelocity", m_velocityGain))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading K_GainVelocity vector of the hand.";
        return false;
    }

    if(!YarpHelper::getYarpVectorFromSearchable(config, "K_GainBuzzMotors", m_buzzMotorsGain))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading K_GainBuzzMotors vector of the hand.";
        return false;
    }



    return true;
}

bool Retargeting::retargetHumanMotionToRobot(){

    return true;
}

bool Retargeting::retargetForceFeedbackFromRobotToHuman(){

    return true;
}

bool Retargeting::retargetVibroTactileFeedbackFromRobotToHuman(){

    return true;
}

bool Retargeting::retargetHapticFeedbackFromRobotToHuman(){

    return true;
}

