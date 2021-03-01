/**
 * @file GloveControlHelper.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#include <limits>

// iDynTree
#include <iDynTree/Core/Utils.h>

#include <GloveControlHelper.hpp>
#include <Utils.hpp>

using namespace HapticGlove;

bool GloveControlHelper::configure(const yarp::os::Searchable& config,
                                   const std::string& name,
                                   const bool& rightHand)
{

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    m_isReady = false;
    m_forceFbDof = 5;
    m_buzzDof = 5;
    m_handNoLinks = 20;
    m_gloveNoLinks = 30;
    m_NoSensors = 20;

    m_isRightHand = rightHand;
    m_desiredBuzzValues.resize(m_buzzDof, 0);
    m_desiredForceValues.resize(m_forceFbDof, 0);
    m_glovePose = Eigen::MatrixXd::Zero(m_gloveNoLinks, 7);
    m_handPose = Eigen::MatrixXd::Zero(m_handNoLinks, 7);

    m_handJointsAngles = Eigen::MatrixXd::Zero(m_handNoLinks, 3);



    yarp::os::Value* jointListYarp;
    if (!config.check("human_joint_list", jointListYarp))
    {
        yError() << "[GloveControlHelper::configure] Unable to find human_joint_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(jointListYarp, m_humanJointNameList))
    {
        yError() << "[GloveControlHelper::configure] Unable to convert human_joint_list list into a "
                    "vector of strings.";
        return false;
    }

    yarp::os::Value* fingerListYarp;
    if (!config.check("human_finger_list", fingerListYarp))
    {
        yError() << "[GloveControlHelper::configure] Unable to find human_finger_list into config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(fingerListYarp, m_humanFingerNameList))
    {
        yError() << "[GloveControlHelper::configure] Unable to convert human_finger_list list into a "
                    "vector of strings.";
        return false;
    }
    m_jointRangeMin.resize(m_humanJointNameList.size(), 0.0);
    m_jointRangeMax.resize(m_humanJointNameList.size(), 0.0);

    return true;
}

bool GloveControlHelper::setFingersForceReference(const yarp::sig::Vector& desiredValue)
{
    if (desiredValue.size() != m_forceFbDof)
    {
        yError() << "[GloveControlHelper::setFingersForceReference] the size of the input "
                    "desired vecotr and the number of haptic force feedbacks are not equal.";
        return false;
    }

    for (size_t i = 0; i < m_forceFbDof; i++)
    {
//        if (desiredValue(i) > 0.0)
            m_desiredForceValues[i] = (int)std::round(std::max(0.0,std::min(desiredValue(i), 40.0))*100/40);
//        else
//            m_desiredForceValues[i] = 0;
    }
    std::cout<<"Force Feedback \n"<<m_desiredForceValues<<std::endl;

    m_glove.SendHaptics(SGCore::Haptics::SG_FFBCmd(m_desiredForceValues));

    return true;
}

bool GloveControlHelper::getFingersForceMeasured(yarp::sig::Vector& measuredValue)
{
    return true;
}

bool GloveControlHelper::getGlovePose(Eigen::MatrixXd& measuredValue)
{

    SGCore::SG::SG_GlovePose glovePose;

    if (!m_glove.GetGlovePose(glovePose))
    {
        yWarning() << "m_glove.GetGlovePose return error.";
        measuredValue = m_glovePose;
        return true;
    }

    int count = 0;
    for (int i = 0; i < glovePose.jointPositions.size(); i++)
    {
        for (int j = 0; j < glovePose.jointPositions[i].size(); j++)
        {

            m_glovePose(count, 0) = glovePose.jointPositions[i][j].x;
            m_glovePose(count, 1) = glovePose.jointPositions[i][j].y;
            m_glovePose(count, 2) = glovePose.jointPositions[i][j].z;

            m_glovePose(count, 3) = glovePose.jointRotations[i][j].x; // wrt to the origin frame
            m_glovePose(count, 4) = glovePose.jointRotations[i][j].y;
            m_glovePose(count, 5) = glovePose.jointRotations[i][j].z;
            m_glovePose(count, 6) = glovePose.jointRotations[i][j].w;
            count++;
        }
    }
    measuredValue = m_glovePose;
    //    yInfo() << "glovePose[ " << i << " ].size(): " << glovePose.jointPositions[i].size();
    //    yInfo() << glovePose.jointPositions[i][j].ToString;
    return true;
}

bool GloveControlHelper::getSensorData(std::vector<float>& measuredValues)
{

    SGCore::SG::SG_SensorData sensorData;
    if (!m_glove.GetSensorData(sensorData))
    {
        yWarning() << "m_glove.GetSensorData return error.";
        measuredValues = m_sensorData;
        return true;
    }

    //    if (!sensorData.IsComplete())
    //    {
    // all the finger, from proximal to distal

    //    }
    m_sensorData = sensorData.GetAngleSequence();
    measuredValues = m_sensorData;
    return true;
}
bool GloveControlHelper::getHandPose(Eigen::MatrixXd& measuredValue)
{
    SGCore::HandProfile profile = SGCore::HandProfile::Default(m_glove.IsRight());
    SGCore::HandPose handPose;
//    SGCore::SG::SG_Solver solver = SGCore::SG::SG_Solver::Interpolation;
    if (!m_glove.GetHandPose(profile, handPose)) //(profile, solver, handPose)
    {
        yWarning() << "m_glove.GetHandPose return error.";
        measuredValue = m_handPose;
        return true;
    }

    int count = 0;
    for (int i = 0; i < handPose.jointPositions.size(); i++)
    {
        for (int j = 0; j < handPose.jointPositions[i].size(); j++)
        {
            m_handPose(count, 0) = handPose.jointPositions[i][j].x;
            m_handPose(count, 1) = handPose.jointPositions[i][j].y;
            m_handPose(count, 2) = handPose.jointPositions[i][j].z;

            m_handPose(count, 3) = handPose.jointRotations[i][j].x; // wrt to the origin frame
            m_handPose(count, 4) = handPose.jointRotations[i][j].y;
            m_handPose(count, 5) = handPose.jointRotations[i][j].z;
            m_handPose(count, 6) = handPose.jointRotations[i][j].w;
            count++;
        }
    }
    measuredValue = m_handPose;

    return true;
}

bool GloveControlHelper::getHandJointsAngles()
{
    SGCore::HandProfile profile = SGCore::HandProfile::Default(m_glove.IsRight());
    SGCore::HandPose handPose;

    if (!m_glove.GetHandPose(profile, handPose))
    {
        yWarning() << "m_glove.GetHandPose return error.";
//        measuredValue = m_handJointsAngles;
        return true;
    }

    // handPose.handAngles.size() --> size is 5 (5 Fingers)
    int count = 0;
    for (int i = 0; i < handPose.jointPositions.size(); i++)
    {

        // handPose.handAngles[i].size() --> size is 3 (3 joints each)

        for (int j = 0; j < handPose.jointPositions[i].size(); j++)
        {

            m_handJointsAngles(count, 0) = handPose.handAngles[i][j].x;
            m_handJointsAngles(count, 1) = handPose.handAngles[i][j].y;
            m_handJointsAngles(count, 2) = handPose.handAngles[i][j].z;
            count++;
        }
    }
    return true;
}

bool GloveControlHelper::getHandJointsAngles(std::vector<double> & jointAngleList)
{
    GloveControlHelper::getHandJointsAngles( );
    jointAngleList.resize(m_humanJointNameList.size(),0.0);

    // thumb
    jointAngleList[0]=m_handJointsAngles(0, 2);
    jointAngleList[1]=m_handJointsAngles(0, 1);
    jointAngleList[2]=m_handJointsAngles(1, 1);
    jointAngleList[3]=m_handJointsAngles(2, 1);

    // m_handJointsAngles(3, :)--> all of them are zero
    // index (3:5)
    jointAngleList[4]=m_handJointsAngles(4, 1);
    jointAngleList[5]=m_handJointsAngles(5, 1);
    jointAngleList[6]=m_handJointsAngles(6, 1);

    // m_handJointsAngles(7, :)--> all of them are zero
    // middle (6:8)
    jointAngleList[7]=m_handJointsAngles(8, 1);
    jointAngleList[8]=m_handJointsAngles(9, 1);
    jointAngleList[9]=m_handJointsAngles(10, 1);

    // m_handJointsAngles(11, :)--> all of them are zero
    // ring (9:11)
    jointAngleList[10]=m_handJointsAngles(12, 1);
    jointAngleList[11]=m_handJointsAngles(13, 1);
    jointAngleList[12]=m_handJointsAngles(14, 1);

    // m_handJointsAngles(15, :)--> all of them are zero
    // pinkie (12:14)
    jointAngleList[13]=m_handJointsAngles(16, 1);
    jointAngleList[14]=m_handJointsAngles(17, 1);
    jointAngleList[15]=m_handJointsAngles(18, 1);

//    yInfo()<<"jointAngleList: "<< jointAngleList;

    return true;
}
bool GloveControlHelper::getHandJointsAngles(Eigen::MatrixXd measuredValue )
{
    measuredValue = m_handJointsAngles;
    return true;
}


bool GloveControlHelper::isConnected()
{
    return m_glove.IsConnected();
}

bool GloveControlHelper::setBuzzMotorsReference(const yarp::sig::Vector& desiredValue)
{
    if (desiredValue.size() != m_buzzDof)
    {
        yError() << "[GloveControlHelper::setVibroTactileJointsReference] the size of the input "
                    "desired vecotr and the number of buzz motors are not equal.";
        return false;
    }
    for (size_t i = 0; i < m_buzzDof; i++)
    {
//        if (desiredValue(i) > 0.0)
//            m_desiredBuzzValues[i] = (int)std::round(std::max(0.0,desiredValue(i)));//(int)std::round(std::max(0.0,std::min(desiredValue(i), 40.0))*100/40);
            m_desiredBuzzValues[i] = (int)std::round(std::max(0.0,std::min(desiredValue(i), 100.0)));
//        else
//            m_desiredBuzzValues[i] = 0;
    }
    // vibrate fingers at percetage intensity, between 0-100, integer numbers
    std::cout<<"Vibrotactile Feedback \n"<<m_desiredBuzzValues<<std::endl;
    m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(m_desiredBuzzValues));


    return true;
}

bool GloveControlHelper::turnOffBuzzMotors()
{
    yInfo() << "[GloveControlHelper::turnOffBuzzMotors]";
    m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off); // turn off all Buzz Motors.
    return true;
}

bool GloveControlHelper::turnForceFeedback()
{
    yInfo() << "[GloveControlHelper::turnForceFeedback]";
    m_glove.SendHaptics(SGCore::Haptics::SG_FFBCmd::off); // turn off all Force Feedback commands.
    return true;

}
const int GloveControlHelper::getNoOfBuzzMotors() const
{
    return m_buzzDof;
}

int GloveControlHelper::getNoOfForceFeedback()
{
    return m_forceFbDof;
}
void GloveControlHelper::close()
{
}

bool GloveControlHelper::setupGlove()
{
    yInfo() << "GloveControlHelper::setupGlove()";

    if (!SGCore::DeviceList::SenseCommRunning()) // Returns true if SenseComm is running.
    {
        yError() << "SenseComm is not running. Please run SenseComm, then try again.";
        return false;
    }
    // GetSenseGlove retrieves the first (connected) Sense Glove it can find. Returns true if one
    // can be found. Additional search parameters can be used.

    if (!SGCore::SG::SenseGlove::GetSenseGlove(m_isRightHand, m_glove))
    {
        yError() << "No sense gloves connected to the system. Ensure the USB connection is "
                    "secure, then try again.";
        return false;
    }

    yInfo() << "Activating " << m_glove.ToString();

    SGCore::SG::SG_GloveInfo gloveModel = m_glove.GetGloveModel();
    yInfo() << "glove model:" << gloveModel.ToString();
    yInfo() << "glove model:" << gloveModel.ToString(false);

    std::vector<double> jointAngleList;
    getHandJointsAngles(jointAngleList);
    for(size_t i=0; i<m_humanJointNameList.size(); i++)
    {
        m_jointRangeMin[i]=    jointAngleList[i];
        m_jointRangeMax[i]=    jointAngleList[i];
    }

    return true;
}

bool GloveControlHelper::stopFeedback()
{
/*
    return m_glove.StopFeedback();
*/
return true;
}

bool GloveControlHelper::setPalmFeedbackThumper(const int desiredValue)
{
    if (desiredValue == 0)
        return m_glove.SendHaptics(SGCore::Haptics::Impact_Thump_100);
    else if (desiredValue == 1)
        return m_glove.SendHaptics(SGCore::Haptics::Object_Grasp_100);
    else
        return m_glove.SendHaptics(SGCore::Haptics::Button_Double_100);
}

int GloveControlHelper::getNoGloveLinks()
{
    return m_gloveNoLinks;
}

int GloveControlHelper::getNoHandLinks()
{
    return m_handNoLinks;
}

int GloveControlHelper::getNoSensors()
{
    return m_NoSensors;
}



void GloveControlHelper::getHumanJointsList( std::vector<std::string>& jointList)const {
    jointList = m_humanJointNameList;
}

void GloveControlHelper::getHumanFingersList(std::vector<std::string>& fingerList)const {
    fingerList = m_humanFingerNameList;
}

bool GloveControlHelper::findHumanMotionRange(){
    yInfo()<<"GloveControlHelper::findHumanMotionRange";

    std::vector<double> jointAngleList;
    getHandJointsAngles(jointAngleList);

    for(size_t i=0; i<m_humanJointNameList.size(); i++)
    {
        m_jointRangeMin[i]=std::min(m_jointRangeMin[i], jointAngleList[i]);
        m_jointRangeMax[i]=std::max(m_jointRangeMax[i], jointAngleList[i]);
    }

    yarp::sig::Vector desiredValue;
    desiredValue.resize(m_buzzDof,40);
//    setBuzzMotorsReference(desiredValue);

return true;
}

void GloveControlHelper::getHumanMotionRange( std::vector<double>& jointRangeMin, std::vector<double>& jointRangeMax)
{
    yInfo()<<"GloveControlHelper::getHumanMotionRange";
    jointRangeMin.resize(m_humanJointNameList.size(), 0.0);
    jointRangeMax.resize(m_humanJointNameList.size(), 0.0);
    for(size_t i=0; i<m_humanJointNameList.size(); i++)
    {
        jointRangeMax[i]=m_jointRangeMax[i];
        jointRangeMin[i]=m_jointRangeMin[i];
    }
    std::cout<<"jointRangeMax"<<jointRangeMax<<std::endl;
    std::cout<<"jointRangeMin"<<jointRangeMin<<std::endl;
}



///// <summary> set the level(s) of force and vibrotactile feedback, with an optional thumper
/// command
///// </summary>
// bool sendhaptics(haptics::sg_ffbcmd ffbcmd,
//                 haptics::sg_buzzcmd buzzcmd,
//                 haptics::sg_thumpercmd thumpercmd = haptics::sg_thumpercmd::none);
//
///// <summary> send a force-feedback command to the sense glove. </summary>
// bool sendhaptics(haptics::sg_ffbcmd ffbcmd);
//
///// <summary> send a vibration command to the sense glove. </summary>
// bool sendhaptics(haptics::sg_buzzcmd buzzcmd);
//
///// <summary> send a thumper command. </summary>
// bool sendhaptics(haptics::sg_thumpercmd thumpercmd);
//
///// <summary> stop all haptic feedback on this device. </summary>
// bool stopfeedback();
