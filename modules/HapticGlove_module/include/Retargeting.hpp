#ifndef RETARGETING_H
#define RETARGETING_H

#include <vector>
#include <yarp/sig/Vector.h>




#include <RobotController_hapticGlove.hpp>
#include <GloveControlHelper.hpp>


class Retargeting{


    yarp::sig::Vector m_totalGain;
    yarp::sig::Vector m_velocityGain;
    yarp::sig::Vector m_buzzMotorsGain;

   const RobotController* m_robotHand;
   const HapticGlove::GloveControlHelper * m_gloveHand;




public:
    Retargeting(const RobotController & robot, const HapticGlove::GloveControlHelper& human);

    bool configure(const yarp::os::Searchable& config, const std::string& name);

    bool retargetHumanMotionToRobot();

    bool retargetForceFeedbackFromRobotToHuman();

    bool retargetVibroTactileFeedbackFromRobotToHuman();

    bool retargetHapticFeedbackFromRobotToHuman();





};

#endif // RETARGETING_H
