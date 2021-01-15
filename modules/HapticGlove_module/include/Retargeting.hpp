#ifndef RETARGETING_H
#define RETARGETING_H

#include <vector>
#include <yarp/sig/Vector.h>

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


    size_t m_noAllAxis;
    size_t m_noBuzzMotors;
    std::vector<std::string>m_robotActuatedAxisNameList;

   std::vector<unsigned> m_humanToRobotMap;
   std::vector<std::string> m_humanJointNameList, m_robotActuatedJointNameList; //names

    std::vector<double> m_humanJointAngles; //values
    yarp::sig::Vector m_robotRefJointAngles; //values
    yarp::sig::Vector m_fingerForceFeedback; //values
    yarp::sig::Vector m_fingerBuzzFeedback; //values

    std::vector<FingerAxisRelation>  m_fingerAxisRelation;

public:
    Retargeting(const size_t noAllAxis,const size_t noBuzzMotors,  const std::vector<std::string>& robotActuatedJointNameList,
                const std::vector<std::string>& robotActuatedAxisNameList, const std::vector<std::string>& humanJointNameList);

    bool configure(const yarp::os::Searchable& config, const std::string& name);

    bool retargetHumanMotionToRobot(const std::vector<double> & humanJointAngles);

    bool retargetForceFeedbackFromRobotToHuman(const yarp::sig::Vector& axisValueError,const yarp::sig::Vector& axisVelocityError );

    bool retargetVibroTactileFeedbackFromRobotToHuman();

    bool retargetHapticFeedbackFromRobotToHuman(const yarp::sig::Vector& axisValueError,const yarp::sig::Vector& axisVelocityError );

    bool mapFromHuman2Robot(std::vector<std::string> humanListName,
                            std::vector<std::string> robotListNames,
                            std::vector<unsigned>& humanToRobotMap);


    bool getRobotJointReferences(yarp::sig::Vector& robotJointReference);

    bool getForceFeedbackToHuman(yarp::sig::Vector& forceFeedbackList);

    bool getVibroTactileFeedbackToHuman(yarp::sig::Vector& buzzFeedbackList);




};

#endif // RETARGETING_H
