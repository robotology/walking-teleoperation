#include <XsensRetargeting.hpp>
#include <Utils.hpp>
#include <sstream>
#include <iterator>

//std::vector<double> WholeBodyRetargeting::split(const std::string& str, const std::string& delim)
//{
//    std::vector<double> tokens;
//    size_t prev = 0, pos = 0;
//    do
//    {
//        pos = str.find(delim, prev);
//        if (pos == std::string::npos) pos = str.length();
//        std::string token =  str.substr(prev, pos-prev);
//        if (!token.empty()) tokens.push_back(std::stod(token));
//        prev = pos + delim.length();
//    }
//    while (pos < str.length() && prev < str.length());
//    return tokens;
//}
WholeBodyRetargeting::WholeBodyRetargeting(){};

WholeBodyRetargeting::~WholeBodyRetargeting(){};

bool WholeBodyRetargeting::configure(yarp::os::ResourceFinder& rf)
{
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[WholeBodyRetargeting::configure] Empty configuration for the OculusModule application.";
        return false;
    }

//    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_dT = rf.check("samplingTime", yarp::os::Value(0.1)).asDouble();

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());


//    m_useXsens = generalOptions.check("useXsens", yarp::os::Value(false)).asBool();
//    yInfo()<<"Teleoperation uses Xsens: "<<m_useXsens;

//    yarp::os::Bottle& config = rf.findGroup("WHOLE_BODY_RETARGETING");


    // initialize minimum jerk trajectory for the whole body

    double smoothingTime;
    if (!YarpHelper::getDoubleFromSearchable(rf, "smoothingTime", smoothingTime))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to find the whole body smoothing time";
        return false;
    }



    yarp::os::Value* axesListYarp;
    if (!rf.check("joints_list", axesListYarp))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to find joints_list into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_robotJointsListNames))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }
    m_actuatedDOFs = m_robotJointsListNames.size();



    m_WBTrajectorySmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_actuatedDOFs, m_dT, smoothingTime);
    yarp::sig::Vector buff(m_actuatedDOFs, 0.0);

    m_WBTrajectorySmoother->init(buff);
    m_jointValues.resize(m_actuatedDOFs,0.0);

    yInfo()<<"WholeBodyRetargeting::configure:  smoothingTime: "<<smoothingTime;
    yInfo()<<"WholeBodyRetargeting::configure:  NoOfJoints: "<<m_actuatedDOFs;


    // check the human joints name list
    yarp::os::Value* humanAxesListYarp;
    if (!rf.check("human_joint_list_stream", humanAxesListYarp))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to find human_joint_list_stream into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(humanAxesListYarp, m_humanJointsListName))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }

    yInfo()<<"Human joints name list: [human joints list] [robot joints list]"<<m_humanJointsListName.size()<<" , "<<m_robotJointsListNames.size();

    for(size_t i=0;i<m_humanJointsListName.size();i++)
    {
        if(i<m_robotJointsListNames.size())
        yInfo()<<"("<<i<<"): "<<m_humanJointsListName[i]<<" , "<<m_robotJointsListNames[i] ;
        else {
            yInfo()<<"("<<i<<"): "<<m_humanJointsListName[i]<<" , --" ;
        }
    }

    std::string portName;
    if (!YarpHelper::getStringFromSearchable(rf, "wholeBodyJointsPort", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_wholeBodyHumanJointsPort.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] " << portName << " port already open.";
        return false;
    }
    //DEL Later
//    if( !yarp::os::Network::connect("/HDE/HumanStateWrapper/state:o","/" + getName() + portName))
//    {
//        yError() << "[OculusModule::configure] could nor connect";
//        return false;
//    }


    if (!YarpHelper::getStringFromSearchable(rf, "controllerPort", portName))
    {
        yError() << "[OculusModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_wholeBodyHumanSmoothedJointsPort.open("/" + getName() + portName))
    {
        yError() << "[OculusModule::configure] Unable to open the port " << portName;
        return false;
    }

    // I should do a maping between two vectors here.

    bool foundMatch=false;
    for(unsigned i=0; i<m_robotJointsListNames.size();i++)
    {
        for(unsigned j=0;j<m_humanJointsListName.size();j++)
        {

            if(m_robotJointsListNames[i]==m_humanJointsListName[j])
            {
                foundMatch=true;
                m_humanToRobotMap.push_back(j);
                break;
            }
        }
        if(!foundMatch){
            yError()<<"not found match for: "<<m_robotJointsListNames[i]<< " , "<<i;;
            return false;
        }
        foundMatch=false;
    }

    yInfo()<<"*** mapped joint names: ****";
    for(size_t i=0;i<m_robotJointsListNames.size();i++)
    {
        yInfo()<<"("<<i<<", "<<m_humanToRobotMap[i]<<"): "<<m_robotJointsListNames[i]<<" , "<<m_humanJointsListName[(m_humanToRobotMap[i])] ;
    }

//     m_tick =std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
//     m_tock =std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
    m_firstIteration=true;

    double jointThreshold;
    if (!YarpHelper::getDoubleFromSearchable(rf, "jointDifferenceThreshold", jointThreshold))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to find the whole body joint difference threshold.";
        return false;
    }
    m_jointDiffThreshold=jointThreshold;

    yInfo()<<" Sampling time  : "<<m_dT;
    yInfo()<<" Smoothing time : "<<smoothingTime;
    yInfo()<<" Joint threshold: "<<m_jointDiffThreshold;


   yInfo()<<" WholeBodyRetargeting::configure done!";
    return true;

}
bool WholeBodyRetargeting::getJointValues()
{
       yarp::os::Bottle* desiredHumanJoints=m_wholeBodyHumanJointsPort.read(false);

    if(desiredHumanJoints == NULL)
    {
       //yError()<< "[WholeBodyRetargeting::getJointValues()] desiredWBJoints size: 0";
       return true;
    }
    //yInfo()<< "[WholeBodyRetargeting::setJointValues()] desiredWBJoints size: "<<desiredHumanJoints->size();
//    printf("Received %s\n", desiredHumanJoints->toString().c_str());
    //humanJointValues=desiredHumanJoints->get(0).toString().c_str();
//    printf("Received %s\n", desiredHumanJoints->get(0).toString().c_str());
  //  std::string humanjointsValueS=desiredHumanJoints->get(0).toString().c_str();
     yarp::os::Value humanjointsValueS=desiredHumanJoints->get(0);
     yarp::os::Bottle* tmpHumanNewJointValues = humanjointsValueS.asList();

//     if (tmpHumanNewJointValues->size() != m_actuatedDOFs)
//     {
//         yError() << "Dummy joint size is not matched to m_actuatedDOFs";
//         return false;
//     }

    yarp::sig::Vector newJointValues;
    newJointValues.resize(m_actuatedDOFs,0.0);

//    yInfo() << "a";

     for (unsigned j = 0; j < m_actuatedDOFs; j++)
     {
         newJointValues(j) = tmpHumanNewJointValues->get(m_humanToRobotMap[j]).asDouble();
         if(!m_firstIteration)
         {
             if( std::abs(newJointValues(j)-m_jointValues(j)) <m_jointDiffThreshold   )
             {   m_jointValues(j)=newJointValues(j);  }
         }
         else
         {
            yInfo()<<"[WholeBodyRetargeting::getJointValues] Whole Body Retargeting Module is Running ...";
             m_firstIteration=false;
             m_jointValues(j)=newJointValues(j);
         }
     }

//     yInfo() << "b";


//    yarp::sig::Vector humanjointsValueD=desiredHumanJoints->get(0).asDouble();
//    yInfo()<<"humanjointsValueS: " <<humanjointsValueS;
//    std::vector<double> humanJointValues=split(humanjointsValueS," ");

//    yInfo()<<"1: "<<m_jointValues[0]<<" ,2: "<<m_jointValues[1]<<" , "<<m_jointValues[2];

//    m_tock =std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
//    yDebug() << "---------- >rate: "<< (m_tock-m_tick).count() << "ms";

//    m_tick =std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());


    //for( unsigned i=0;i<m_actuatedDOFs;i++)
    //m_jointValues(i)=humanJointValues[m_humanToRobotMap[i]];

//    yInfo() << "c";

    return true;
}

bool WholeBodyRetargeting::getSmoothedJointValues(yarp::sig::Vector& smoothedJointValues)
{


    m_WBTrajectorySmoother->computeNextValues(m_jointValues);
    smoothedJointValues = m_WBTrajectorySmoother->getPos();
//    auto tock = std::chrono::high_resolution_clock::now();
//    yDebug() << "IK took"
//          << std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count() << "ms";


    return true;}

double WholeBodyRetargeting::getPeriod()
{

     return m_dT;
}

bool WholeBodyRetargeting::updateModule()
{

//    std::chrono::milliseconds tick=std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
    getJointValues();

    if(m_wholeBodyHumanJointsPort.isClosed()) {
        yError() << "[WholeBodyRetargeting::updateModule] m_wholeBodyHumanJointsPort port is closed";
        return false;
    }
    yarp::sig::Vector& refValues = m_wholeBodyHumanSmoothedJointsPort.prepare();
    getSmoothedJointValues(refValues);
    m_wholeBodyHumanSmoothedJointsPort.write();

//    std::chrono::milliseconds tock=std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
//    yDebug() << "update rate: "<< (tock-tick).count() << "ms";



    return true;
}

bool WholeBodyRetargeting::close()
{
    return true;

}

