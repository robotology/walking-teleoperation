#include <WholeBodyRetargeting.hpp>
#include <Utils.hpp>

WholeBodyRetargeting::WholeBodyRetargeting(){};

WholeBodyRetargeting::~WholeBodyRetargeting(){};

bool WholeBodyRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    // check if the configuration file is empty

    if (config.isNull())
    {
        yError() << "[WholeBodyRetargeting::configure] Empty configuration for whole body retargeting.";
        return false;
    }

    m_controlHelper = std::make_unique<RobotControlHelper>();
    if (!m_controlHelper->configure(config, name, true))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to configure the whole body helper";
        return false;
    }

    // initialize minimum jerk trajectory for the whole body
    double samplingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to find the whole body sampling time";
        return false;
    }


    double smoothingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "smoothingTime", smoothingTime))
    {
        yError() << "[WholeBodyRetargeting::configure] Unable to find the whole body smoothing time";
        return false;
    }

    unsigned int NoOfJoints=static_cast<unsigned int>(m_controlHelper->getDoFs());
    m_WBTrajectorySmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(NoOfJoints, samplingTime, smoothingTime);
    yarp::sig::Vector buff(NoOfJoints, 0.0);

    m_WBTrajectorySmoother->init(buff);
    m_jointValues.resize(NoOfJoints,0.0);

    yInfo()<<"WholeBodyRetargeting::configure:  smoothingTime: "<<smoothingTime;
    yInfo()<<"WholeBodyRetargeting::configure:  NoOfJoints: "<<NoOfJoints;


    // check the human joints name list
    yarp::os::Value* axesListYarp;
    if (!config.check("human_joint_list_stream", axesListYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find joints_list into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_humanJointsListName))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }

    yInfo()<<"Human joints name list:";

    for(int i=0;i<m_humanJointsListName.size();i++)
    {
        yInfo()<<"("<<i<<"): "<<m_humanJointsListName[i]<<" , ";
    }


    return true;

}
bool WholeBodyRetargeting::setJointValues(const yarp::sig::Vector& jointValues)
{
    size_t NoOfJoints=m_jointValues.size();
    if(NoOfJoints!=jointValues.size())
    {
        yError() << "[WholeBodyRetargeting::setJointValues] the number of joints in the config file and incoming data are not equal";
        return false;
    }
    for (size_t i=0;i<NoOfJoints;i++)
    {
        m_jointValues(i)=jointValues(i);
    }
    return true;
}

bool WholeBodyRetargeting::getSmoothedJointValues(yarp::sig::Vector&  smoothedJointValues)
{

    m_WBTrajectorySmoother->computeNextValues(m_jointValues);
    smoothedJointValues = m_WBTrajectorySmoother->getPos();


    return true;
}
