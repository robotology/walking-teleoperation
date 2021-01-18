#include <Retargeting.hpp>
#include <Utils.hpp>
#include <algorithm>

Retargeting::Retargeting(const size_t noAllAxis,const size_t noActuatedAxis, const size_t noBuzzMotors,  const std::vector<std::string>& robotActuatedJointNameList,
                         const std::vector<std::string>& robotActuatedAxisNameList, const std::vector<std::string>& humanJointNameList) {

m_noAllAxis= noAllAxis;
m_noActuatedAxis= noActuatedAxis;
m_robotActuatedJointNameList= robotActuatedJointNameList;
m_robotActuatedAxisNameList= robotActuatedAxisNameList;

m_noBuzzMotors=noBuzzMotors;
m_humanJointNameList=humanJointNameList;
}

bool Retargeting::configure(const yarp::os::Searchable& config, const std::string& name){
    yarp::sig::Vector totalGainAllAxis,  velocityGainAllAxis;
    totalGainAllAxis.resize(m_noAllAxis, 0.0);
    velocityGainAllAxis.resize(m_noAllAxis, 0.0);

//    m_totalGain.resize(m_noActuatedAxis, 0.0);
//    m_velocityGain.resize(m_noActuatedAxis, 0.0);
    m_retargetingScaling.resize(m_humanJointNameList.size(), 0.0);
    m_retargetingBias.resize(m_humanJointNameList.size(), 0.);
    m_fingerBuzzMotorsGain.resize(m_noBuzzMotors, 0.0);

    yarp::os::Value* robotAllAxisNameListYarp;
    yarp::os::Value* robotActuatedAxisNameListYarp;
    std::vector<std::string> robotAllAxisNameList, robotActuatedAxisNameList;
    if (!config.check("all_axis_list", robotAllAxisNameListYarp))
    {
        yError() << "[Retargeting::configure] Unable to find all_axis_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(robotAllAxisNameListYarp, robotAllAxisNameList))
    {
        yError() << "[GloveControlHelper::configure] Unable to convert all_axis_list list into a "
                    "vector of strings.";
        return false;
    }

    if (!config.check("axis_list", robotActuatedAxisNameListYarp))
    {
        yError() << "[Retargeting::configure] Unable to find axis_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(robotActuatedAxisNameListYarp, robotActuatedAxisNameList))
    {
        yError() << "[GloveControlHelper::configure] Unable to convert axis_list list into a "
                    "vector of strings.";
        return false;
    }

    if(!YarpHelper::getYarpVectorFromSearchable(config, "K_GainTotal", totalGainAllAxis))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading K_GainTotal vector of the hand.";
        return false;
    }
    if(!YarpHelper::getYarpVectorFromSearchable(config, "K_GainVelocity", velocityGainAllAxis))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading K_GainVelocity vector of the hand.";
        return false;
    }

    getCustomSetIndecies(robotAllAxisNameList, robotActuatedAxisNameList, totalGainAllAxis, m_totalGain);
    getCustomSetIndecies(robotAllAxisNameList, robotActuatedAxisNameList, velocityGainAllAxis, m_velocityGain);

    if(!YarpHelper::getYarpVectorFromSearchable(config, "K_GainBuzzMotors", m_fingerBuzzMotorsGain))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading K_GainBuzzMotors vector of the hand.";
        return false;
    }

    // ****
    if(!YarpHelper::getYarpVectorFromSearchable(config, "human_to_robot_joint_anlges_scaling", m_retargetingScaling))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading human_to_robot_joint_anlges_scaling vector of the hand.";
        return false;
    }

    if(!YarpHelper::getYarpVectorFromSearchable(config, "human_to_robot_joint_anlges_bias", m_retargetingBias))
    {
        yError() << "[Retargeting::configure] Initialization failed while reading human_to_robot_joint_anlges_bias vector of the hand.";
        return false;
    }

    yInfo()<<"K_GainTotal "<< m_totalGain.toString();
    yInfo()<<"K_GainVelocity "<< m_velocityGain.toString();
    yInfo()<<"K_GainBuzzMotors "<< m_fingerBuzzMotorsGain.toString();
    yInfo()<<"human_to_robot_joint_anlges_scaling "<< m_retargetingScaling.toString();
    yInfo()<<"human_to_robot_joint_anlges_bias "<< m_retargetingBias.toString();

    /////////////
    ///////////// Get human and robot joint list and find the mapping between them
    /////////////

    mapFromHuman2Robot(m_humanJointNameList, m_robotActuatedJointNameList, m_humanToRobotMap);

    m_robotRefJointAngles.resize(m_robotActuatedJointNameList.size(), 0.0); ;//.resize(m_humanJointNameList.size(),0.0);

    //////////

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


            auto elementAxis = std::find(std::begin(m_robotActuatedAxisNameList), std::end(m_robotActuatedAxisNameList), *it_axis );
            if (elementAxis != std::end(m_robotActuatedAxisNameList))
            {
                size_t indexAxis= elementAxis-m_robotActuatedAxisNameList.begin();
                tmpObj.m_robotActuatedAxisIndex.push_back(indexAxis);
            }
        }
        m_fingerAxisRelation.push_back(tmpObj);
    }
    return true;
}

bool Retargeting::retargetHumanMotionToRobot(const std::vector<double> & humanJointAngles){

    if(humanJointAngles.size()!=m_humanJointNameList.size())
    {
        yError()<<"[Retargeting::retargetHumanMotionToRobot] the size of human joint name and angles are different.";
        return false;
    }
    for (size_t i= 0; i<m_robotActuatedJointNameList.size();++i)
    {
        m_robotRefJointAngles(i)=m_retargetingScaling(m_humanToRobotMap[i])*humanJointAngles[m_humanToRobotMap[i]] + m_retargetingBias(m_humanToRobotMap[i]);
    }


    return true;
}

bool Retargeting::retargetForceFeedbackFromRobotToHuman(const yarp::sig::Vector& axisValueError,const yarp::sig::Vector& axisVelocityError ){

    std::cout<<"force feedback\n";
    for (size_t i=0; i<m_fingerAxisRelation.size();i++)
    {
        std::cout<<m_fingerAxisRelation[i].fingerName<<" ";
       m_fingerForceFeedback(i)=0.0;
       std::cout<< i<<":  ";
       for(int j=0; j <m_fingerAxisRelation[i].m_robotActuatedAxisIndex.size();j++)
       {
           // related actuated axis index
           size_t Index=m_fingerAxisRelation[i].m_robotActuatedAxisIndex[j];
           std::cout<< " [ "<<Index<<" ] "<< m_totalGain(Index)<< " "<<  axisValueError(Index) << " "<<  m_velocityGain(Index) << " "<<  axisVelocityError(Index)<< " ";
           m_fingerForceFeedback(i)+= m_totalGain(Index) * ( axisValueError(Index) + m_velocityGain(Index) * axisVelocityError(Index) );
           std::cout<< " --> "<<  m_totalGain(Index) * ( axisValueError(Index) + m_velocityGain(Index) * axisVelocityError(Index) );
       }
       std::cout<< "\n";
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

bool Retargeting::retargetHapticFeedbackFromRobotToHuman(const yarp::sig::Vector& axisValueError,const yarp::sig::Vector& axisVelocityError )
{

    retargetForceFeedbackFromRobotToHuman(axisValueError, axisVelocityError);
    retargetVibroTactileFeedbackFromRobotToHuman();

    return true;
}

bool Retargeting::getRobotJointReferences(yarp::sig::Vector& robotJointReference){
    robotJointReference = m_robotRefJointAngles;
    return true;
}

bool Retargeting::getForceFeedbackToHuman(yarp::sig::Vector& forceFeedbackList){

    forceFeedbackList = m_fingerForceFeedback;
    return true;
}

bool Retargeting::getVibroTactileFeedbackToHuman(yarp::sig::Vector& buzzFeedbackList){

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
            yError() << "[Retargeting::mapFromHuman2Robot] not found match for: "
                     << robotListNames[i] << " , " << i;
            return false;
        }
        foundMatch = false;
    }

    yInfo() << "*** mapped joint names:  human --> robot ****";
    for (size_t i = 0; i < robotListNames.size(); i++)
    {
        yInfo() << "(" << i << ", " << humanToRobotMap[i] << "): " << robotListNames[i]
                   << " , " << humanListName[(humanToRobotMap[i])];
    }

    return true;
}


bool Retargeting::getCustomSetIndecies( const std::vector<std::string>& allListName,
                                        const std::vector<std::string>& customListNames,
                                        const yarp::sig::Vector& allListVector,
                                        yarp::sig::Vector& customListVector)
{
    customListVector.clear();
    if(allListName.empty())
    {
        yInfo()<< "[Retargeting::getCustomSetIndecies] all list name is empty.";
        return false;
    }

    bool foundMatch = false;
    for (unsigned i = 0; i < customListNames.size(); i++)
    {
        for (unsigned j = 0; j < allListName.size(); j++)
        {

            if (customListNames[i] == allListName[j])
            {
                foundMatch = true;
                customListVector.push_back(allListVector[j]);
                break;
            }
        }
        if (!foundMatch)
        {
            yError() << "[Retargeting::getCustomSetIndecies] not found match for: "
                     << customListNames[i] << " , " << i;
            return false;
        }
        foundMatch = false;
    }

    if (customListNames.size()!=customListVector.size())
    {
        yError()<<"[Retargeting::getCustomSetIndecies] customListName and customListVector should have similar size";
        return false;
    }

    yInfo() << "*** custom List Vector:****";
    for (size_t i = 0; i < customListNames.size(); i++)
    {
        yInfo() << " (" << i << ", " << customListNames[i] << "): " << customListVector[i];
    }

    return true;
}
