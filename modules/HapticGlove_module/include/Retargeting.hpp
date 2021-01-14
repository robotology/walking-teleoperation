#ifndef RETARGETING_H
#define RETARGETING_H

#include <vector>
#include <yarp/sig/Vector.h>




#include <RobotController_hapticGlove.hpp>
#include <GloveControlHelper.hpp>


struct FingerAxisRelation
{
    std::string fingerName;
    std::vector<size_t> m_robotActuatedAxisIndex;

};

class Retargeting{


    yarp::sig::Vector m_totalGain;
    yarp::sig::Vector m_velocityGain;
    yarp::sig::Vector m_fingerBuzzMotorsGain;

    yarp::sig::Vector m_retargetingScaling;
    yarp::sig::Vector m_retargetingBias;


   const RobotController* m_robotHand;
   const HapticGlove::GloveControlHelper * m_gloveHand;

   std::vector<unsigned> m_humanToRobotMap;
   std::vector<std::string> m_humanJointNameList, m_robotActuatedJointNameList; //names

    std::vector<double> m_humanJointAngles; //values
    yarp::sig::Vector m_robotRefJointAngles; //values
    yarp::sig::Vector m_fingerForceFeedback; //values
    yarp::sig::Vector m_buzzFeedback; //values

    std::vector<FingerAxisRelation>  m_fingerAxisRelation;

public:
    Retargeting(const RobotController & robot, const HapticGlove::GloveControlHelper& human);

    bool configure(const yarp::os::Searchable& config, const std::string& name);

    bool retargetHumanMotionToRobot();

    bool retargetForceFeedbackFromRobotToHuman(yarp::sig::Vector axisValueError,yarp::sig::Vector axisVelocityError );

    bool retargetVibroTactileFeedbackFromRobotToHuman();

    bool retargetHapticFeedbackFromRobotToHuman(yarp::sig::Vector axisValueError,yarp::sig::Vector axisVelocityError );

    bool mapFromHuman2Robot(std::vector<std::string> humanListName,
                            std::vector<std::string> robotListNames,
                            std::vector<unsigned>& humanToRobotMap);


    bool getRobotJointReferences(yarp::sig::Vector robotJointReference);



};

#endif // RETARGETING_H
