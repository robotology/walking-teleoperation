#include <Retargeting.hpp>
#include <Utils.hpp>

Retargeting::    Retargeting(const RobotController & robot, const HapticGlove::GloveControlHelper& human): m_robotHand(&robot),
m_gloveHand(&human)
{



}

bool Retargeting::configure(const yarp::os::Searchable& config, const std::string& name){

    m_totalGain.resize(m_robotHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);
    m_velocityGain.resize(m_robotHand->controlHelper()->getNumberOfActuatedAxis(), 0.0);

    m_fingerBuzzMotorsGain.resize(m_gloveHand->getNoOfBuzzMotors(), 0.0);

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

    if(!YarpHelper::getYarpVectorFromSearchable(config, "K_GainBuzzMotors", m_fingerBuzzMotorsGain))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading K_GainBuzzMotors vector of the hand.";
        return false;
    }

    // ****
    if(!YarpHelper::getYarpVectorFromSearchable(config, "human_to_robot_joint_anlges_scaling", m_retargetingScaling))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading m_retargetingScaling vector of the hand.";
        return false;
    }

    if(!YarpHelper::getYarpVectorFromSearchable(config, "human_to_robot_joint_anlges_bias", m_retargetingBias))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading human_to_robot_joint_anlges_bias vector of the hand.";
        return false;
    }

    /////////////
    ///////////// Get human and robot joint list and find the mapping between them
    /////////////

    m_gloveHand->getHumanJointsList(m_humanJointNameList);

    m_robotHand->controlHelper()->getActuatedJointNameList(m_robotActuatedJointNameList);

    mapFromHuman2Robot(m_humanJointNameList, m_robotActuatedJointNameList, m_humanToRobotMap);

    m_humanJointAngles.resize(m_humanJointNameList.size(),0.0);
    m_robotRefJointAngles.resize(m_robotActuatedJointNameList.size(), 0.0); ;//.resize(m_humanJointNameList.size(),0.0);

    //////////

    std::vector<std::string>robotActuatedAxisNameList;
    m_robotHand->controlHelper()->getActuatedAxisNameList(robotActuatedAxisNameList);

    yarp::os::Value* humanFingersListYarp;
    std::vector<std::string> humanFingersList;
    if (!config.check("human_finger_list", humanFingersListYarp))
    {
        yError() << "[Retargeting::configure] Unable to find human_finger_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(humanFingersListYarp, humanFingersList))
    {
        yError() << "[GloveControlHelper::configure] Unable to convert human_finger_list list into a "
                    "vector of strings.";
        return false;
    }

    m_fingerForceFeedback.resize(humanFingersList.size(),0.0);
    m_fingerBuzzFeedback.resize(humanFingersList.size(),0.0);

    for (std::vector<std::string>::iterator it = humanFingersList.begin() ; it != humanFingersList.end(); ++it)
    {
        FingerAxisRelation tmpObj;
        tmpObj.fingerName=*it;

        yarp::os::Value* axisFingerListYarp;
        std::vector<std::string> axisFingerList;
        if (!config.check(*it,axisFingerListYarp))
        {
            yError() << "[Retargeting::configure] Unable to find "<<*it<<" into config file.";
            return false;
        }
        if (!YarpHelper::yarpListToStringVector(axisFingerListYarp, axisFingerList))
        {
            yError() << "[GloveControlHelper::configure] Unable to convert "<<*it<<" list into a "
                        "vector of strings.";
            return false;
        }


        for (std::vector<std::string>::iterator it_axis = axisFingerList.begin() ; it_axis!= axisFingerList.end(); ++it_axis)
        {


            auto elementAxis = std::find(std::begin(robotActuatedAxisNameList), std::end(robotActuatedAxisNameList), *it_axis );
            if (elementAxis != std::end(robotActuatedAxisNameList))
            {
                size_t indexAxis= elementAxis-robotActuatedAxisNameList.begin();
                tmpObj.m_robotActuatedAxisIndex.push_back(indexAxis);
            }
        }
        m_fingerAxisRelation.push_back(tmpObj);
    }
    return true;
}

bool Retargeting::retargetHumanMotionToRobot(){

    m_gloveHand->getHandJointsAngles(m_humanJointAngles);

    for (size_t i= 0; i<m_robotActuatedJointNameList.size();++i)
    {
        m_robotRefJointAngles(i)=m_retargetingScaling(m_humanToRobotMap[i])*m_humanJointAngles[m_humanToRobotMap[i]] + m_retargetingBias(m_humanToRobotMap[i]);
    }

    return true;
}

bool Retargeting::retargetForceFeedbackFromRobotToHuman(yarp::sig::Vector axisValueError,yarp::sig::Vector axisVelocityError ){


    for (size_t i=0; i<m_fingerAxisRelation.size();i++)
    {

       m_fingerForceFeedback(i)=0.0;
       size_t Index; // related actuated axis index
       for(int j=0; j <m_fingerAxisRelation[i].m_robotActuatedAxisIndex.size();j++)
       {
           Index=m_fingerAxisRelation[i].m_robotActuatedAxisIndex[j];
           m_fingerForceFeedback(i)+= m_totalGain(Index) * ( axisValueError(Index) + m_velocityGain(Index) * axisVelocityError(Index) );
       }

    }
    return true;
}

bool Retargeting::retargetVibroTactileFeedbackFromRobotToHuman()
{

    for (size_t i=0; i<m_fingerAxisRelation.size();i++)
    {
        m_fingerBuzzFeedback(i)=m_fingerForceFeedback(i)*m_fingerBuzzMotorsGain(i);
    }
    return true;
}

bool Retargeting::retargetHapticFeedbackFromRobotToHuman(yarp::sig::Vector axisValueError,yarp::sig::Vector axisVelocityError )
{

    retargetForceFeedbackFromRobotToHuman(axisValueError, axisVelocityError);
    retargetVibroTactileFeedbackFromRobotToHuman();

    return true;
}

bool Retargeting::getRobotJointReferences(yarp::sig::Vector robotJointReference){

    robotJointReference = m_robotRefJointAngles;
    return true;
}

bool Retargeting::getForceFeedbackToHuman(yarp::sig::Vector forceFeedbackList){

    forceFeedbackList = m_fingerForceFeedback;
    return true;
}

bool Retargeting::getVibroTactileFeedbackToHuman(yarp::sig::Vector buzzFeedbackList){

    buzzFeedbackList = m_fingerBuzzFeedback;
    return true;
}



bool Retargeting::mapFromHuman2Robot( std::vector<std::string> humanListName,
                                           std::vector<std::string> robotListNames,
                                           std::vector<unsigned>& humanToRobotMap)
{
    if (!humanToRobotMap.empty())
    {
        humanToRobotMap.clear();
    }

    bool foundMatch = false;
    for (unsigned i = 0; i < robotListNames.size(); i++)
    {
        for (unsigned j = 0; j < humanListName.size(); j++)
        {

            if (robotListNames[i] == humanListName[j])
            {
                foundMatch = true;
                humanToRobotMap.push_back(j);
                break;
            }
        }
        if (!foundMatch)
        {
            yError() << "[XsensRetargeting::impl::mapJointsHDE2CONTROLLER] not found match for: "
                     << robotListNames[i] << " , " << i;
            return false;
        }
        foundMatch = false;
    }

    yInfo() << "*** mapped joint names: ****";
    for (size_t i = 0; i < robotListNames.size(); i++)
    {
        yInfo() << "(" << i << ", " << humanToRobotMap[i] << "): " << robotListNames[i]
                   << " , " << humanListName[(humanToRobotMap[i])];
    }

    return true;
}


