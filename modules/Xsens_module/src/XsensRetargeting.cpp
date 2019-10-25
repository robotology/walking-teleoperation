#include <Utils.hpp>
#include <XsensRetargeting.hpp>
#include <iterator>
#include <sstream>

class XsensRetargeting::impl
{
public:
    /*
     * map the joint values (order) coming from HDE to the controller order
     * @param robotJointsListNames list of the joint names used in controller
     * @param humanJointsListName list of the joint names received from the HDE
     * (human-dynamics-estimation repository)
     * @param humanToRobotMap the container for mapping of the human joints to the robot ones
     * @return true in case of success and false otherwise
     */
    bool mapJointsHDE2Controller(std::vector<std::string> robotJointsListNames,
                                 std::vector<std::string> humanJointsListName,
                                 std::vector<unsigned>& humanToRobotMap);
};

XsensRetargeting::XsensRetargeting()
    : pImpl{new impl()} {};

XsensRetargeting::~XsensRetargeting(){};

bool XsensRetargeting::configure(yarp::os::ResourceFinder& rf)
{
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[XsensRetargeting::configure] Empty configuration for the OculusModule "
                    "application.";
        return false;
    }

    // get the period
    m_dT = rf.check("samplingTime", yarp::os::Value(0.1)).asDouble();

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[XsensRetargeting::configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    // initialize minimum jerk trajectory for the whole body

    double smoothingTime;
    if (!YarpHelper::getDoubleFromSearchable(rf, "smoothingTime", smoothingTime))
    {
        yError() << "[XsensRetargeting::configure] Unable to find the whole body smoothing time";
        return false;
    }

    yarp::os::Value* axesListYarp;
    if (!rf.check("joints_list", axesListYarp))
    {
        yError() << "[XsensRetargeting::configure] Unable to find joints_list into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_robotJointsListNames))
    {
        yError() << "[XsensRetargeting::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }
    m_actuatedDOFs = m_robotJointsListNames.size();

    m_WBTrajectorySmoother
        = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_actuatedDOFs, m_dT, smoothingTime);
    yarp::sig::Vector buff(m_actuatedDOFs, 0.0);

    m_WBTrajectorySmoother->init(buff);
    m_jointValues.resize(m_actuatedDOFs, 0.0);

    yInfo() << "XsensRetargeting::configure:  smoothingTime: " << smoothingTime;
    yInfo() << "XsensRetargeting::configure:  NoOfJoints: " << m_actuatedDOFs;

    std::string portName;
    if (!YarpHelper::getStringFromSearchable(rf, "wholeBodyJointsPort", portName))
    {
        yError() << "[XsensRetargeting::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_wholeBodyHumanJointsPort.open("/" + getName() + portName))
    {
        yError() << "[XsensRetargeting::configure] " << portName << " port already open.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "controllerPort", portName))
    {
        yError() << "[XsensRetargeting::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_wholeBodyHumanSmoothedJointsPort.open("/" + getName() + portName))
    {
        yError() << "[XsensRetargeting::configure] Unable to open the port " << portName;
        return false;
    }

    m_firstIteration = true;

    double jointThreshold;
    if (!YarpHelper::getDoubleFromSearchable(rf, "jointDifferenceThreshold", jointThreshold))
    {
        yError() << "[XsensRetargeting::configure] Unable to find the whole body joint difference "
                    "threshold.";
        return false;
    }
    m_jointDiffThreshold = jointThreshold;

    yInfo() << "[XsensRetargeting::configure]"
            << " Sampling time  : " << m_dT;
    yInfo() << "[XsensRetargeting::configure]"
            << " Smoothing time : " << smoothingTime;
    yInfo() << "[XsensRetargeting::configure]"
            << " Joint threshold: " << m_jointDiffThreshold;

    yInfo() << " [XsensRetargeting::configure] done!";
    return true;
}
bool XsensRetargeting::getJointValues()
{
    yarp::os::Bottle* desiredHumanJoints = m_wholeBodyHumanJointsPort.read(false);

    if (desiredHumanJoints == NULL)
    {
        return true;
    }

    // in the msg thrift the first list is the joint names [get(0)], second the joint values
    // [get(1)]

    yarp::os::Value humanjointsValues = desiredHumanJoints->get(1);
    yarp::os::Bottle* tmpHumanNewJointValues = humanjointsValues.asList();

    yarp::sig::Vector newJointValues;
    newJointValues.resize(m_actuatedDOFs, 0.0);

    if (!m_firstIteration)
    {
        for (unsigned j = 0; j < m_actuatedDOFs; j++)
        {
            newJointValues(j) = tmpHumanNewJointValues->get(m_humanToRobotMap[j]).asDouble();
            // check for the spikes in joint values
            if (std::abs(newJointValues(j) - m_jointValues(j)) < m_jointDiffThreshold)
            {
                m_jointValues(j) = newJointValues(j);
            } else
            {
                yWarning() << "spike in data: joint : " << j << " , " << m_robotJointsListNames[j]
                           << " ; old data: " << m_jointValues(j)
                           << " ; new data:" << newJointValues(j);
            }
        }
    } else
    {
        yInfo() << "[XsensRetargeting::getJointValues] Xsens Retargeting Module is Running ...";
        m_firstIteration = false;

        /* We should do a maping between two vectors here: human and robot joint vectors, since
         their order are not the same! */

        // check the human joints name list
        yarp::os::Value humanjointsNames = desiredHumanJoints->get(0);
        yarp::os::Bottle* tmpHumanNewJointNames = humanjointsNames.asList();
        if (tmpHumanNewJointNames->isNull())
        {
            yError() << "[XsensRetargeting::getJointValues()] Human joints name list is empty";
            return false;
        }

        for (unsigned j = 0; j < tmpHumanNewJointNames->size(); j++)
        {
            m_humanJointsListName.push_back(tmpHumanNewJointNames->get(j).asString());
        }

        /* print human and robot joint name list */
        yInfo() << "Human joints name list: [human joints list] [robot joints list]"
                << m_humanJointsListName.size() << " , " << m_robotJointsListNames.size();

        for (size_t i = 0; i < m_humanJointsListName.size(); i++)
        {
            if (i < m_robotJointsListNames.size())
                yInfo() << "(" << i << "): " << m_humanJointsListName[i] << " , "
                        << m_robotJointsListNames[i];
            else
            {
                yInfo() << "(" << i << "): " << m_humanJointsListName[i] << " , --";
            }
        }
        /* find the map between the human and robot joint list orders*/
        if (!pImpl->mapJointsHDE2Controller(
                m_robotJointsListNames, m_humanJointsListName, m_humanToRobotMap))
        {
            yError() << "[XsensRetargeting::getJointValues()] mapping is not possible";
            return false;
        }
        if (m_humanToRobotMap.size() == 0)
        {
            yError() << "[XsensRetargeting::getJointValues()] m_humanToRobotMap.size is zero";
        }

        /* fill the robot joint list values*/
        for (unsigned j = 0; j < m_actuatedDOFs; j++)
        {

            newJointValues(j) = tmpHumanNewJointValues->get(m_humanToRobotMap[j]).asDouble();
            m_jointValues(j) = newJointValues(j);
            yInfo() << " robot initial joint value: (" << j << "): " << m_jointValues[j];
        }
        m_WBTrajectorySmoother->init(m_jointValues);
    }

    yInfo() << "joint [0]: " << m_robotJointsListNames[0] << " : " << newJointValues(0) << " , "
            << m_jointValues(0);

    return true;
}

bool XsensRetargeting::getSmoothedJointValues(yarp::sig::Vector& smoothedJointValues)
{
    m_WBTrajectorySmoother->computeNextValues(m_jointValues);
    smoothedJointValues = m_WBTrajectorySmoother->getPos();

    return true;
}

double XsensRetargeting::getPeriod()
{
    return m_dT;
}

bool XsensRetargeting::updateModule()
{

    getJointValues();

    if (m_wholeBodyHumanJointsPort.isClosed())
    {
        yError() << "[XsensRetargeting::updateModule] m_wholeBodyHumanJointsPort port is closed";
        return false;
    }

    if (!m_firstIteration)
    {

        yarp::sig::Vector& refValues = m_wholeBodyHumanSmoothedJointsPort.prepare();
        getSmoothedJointValues(refValues);
        m_wholeBodyHumanSmoothedJointsPort.write();
    }

    return true;
}

bool XsensRetargeting::close()
{
    return true;
}
bool XsensRetargeting::impl::mapJointsHDE2Controller(std::vector<std::string> robotJointsListNames,
                                                     std::vector<std::string> humanJointsListName,
                                                     std::vector<unsigned>& humanToRobotMap)
{
    if (!humanToRobotMap.empty())
    {
        humanToRobotMap.clear();
    }

    bool foundMatch = false;
    for (unsigned i = 0; i < robotJointsListNames.size(); i++)
    {
        for (unsigned j = 0; j < humanJointsListName.size(); j++)
        {

            if (robotJointsListNames[i] == humanJointsListName[j])
            {
                foundMatch = true;
                humanToRobotMap.push_back(j);
                break;
            }
        }
        if (!foundMatch)
        {
            yError() << "[XsensRetargeting::impl::mapJointsHDE2CONTROLLER] not found match for: "
                     << robotJointsListNames[i] << " , " << i;
            return false;
        }
        foundMatch = false;
    }

    yInfo() << "*** mapped joint names: ****";
    for (size_t i = 0; i < robotJointsListNames.size(); i++)
    {
        yInfo() << "(" << i << ", " << humanToRobotMap[i] << "): " << robotJointsListNames[i]
                << " , " << humanJointsListName[(humanToRobotMap[i])];
    }

    return true;
}
