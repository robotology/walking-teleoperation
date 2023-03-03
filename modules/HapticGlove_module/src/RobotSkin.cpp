/**
 * @file RobotSkin.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

// std
#include <math.h>

// teleoperation
#include <ControlHelper.hpp>
#include <RobotSkin.hpp>
#include <Utils.hpp>

// yarp
#include <yarp/os/Property.h>

using namespace HapticGlove;

RobotSkin::RobotSkin(){};

bool RobotSkin::configure(const yarp::os::Searchable& config,
                          const std::string& name,
                          const bool& rightHand)
{
    m_rightHand = rightHand;
    m_logPrefix = "RobotSkin::";
    m_logPrefix += m_rightHand ? "RightHand:: " : "LeftHand:: ";
    std::string portPrefix = "/";
    portPrefix += m_rightHand ? "RightHand" : "LeftHand";

    m_samplingTime = config.check("samplingTime", yarp::os::Value(0.01)).asFloat64();

    std::vector<std::string> robotFingerNameList;

    if (!YarpHelper::getVectorFromSearchable(config, "robot_finger_list", robotFingerNameList))
    {
        yError() << m_logPrefix << "unable to get human_finger_list from the config file.";
        return false;
    }

    m_tactileWorkingThreshold
        = config.check("tactileWorkingThreshold ", yarp::os::Value(0.0001)).asFloat64();

    m_tactileUpdateThreshold
        = config.check("tactileUpdateThreshold ", yarp::os::Value(-1.0)).asFloat64();

    m_noFingers = robotFingerNameList.size();
    m_totalNoTactile = 0;

    m_areFingersInContact.resize(m_noFingers, false);
    m_fingersInContactTimer.resize(m_noFingers, 0.0);
    m_fingersLastElementInContact.resize(m_noFingers, 0.0);
    m_areFingersContactChanges.resize(m_noFingers, false);
    m_areTactileSensorsWorking.resize(m_noFingers, false);

    m_fingersVibrotactileAbsoluteFeedback.resize(m_noFingers, 0.0);
    m_fingersVibrotactileDerivativeFeedback.resize(m_noFingers, 0.0);
    m_fingersVibrotactileTotalFeedback.resize(m_noFingers, 0.0);
    m_fingersContactStrength.resize(m_noFingers, 0.0);
    m_fingersContactStrengthDerivate.resize(m_noFingers, 0.0);
    m_fingersContactStrengthDerivateSmoothed.resize(m_noFingers, 0.0);

    // raw tactile sensors
    size_t noTactileSensors = config.check("noTactileSensors", yarp::os::Value(192)).asInt64();
    m_fingertipRawTactileFeedbacksYarpVector.resize(noTactileSensors);
    m_fingertipRawTactileFeedbacksStdVector.resize(noTactileSensors);

    // open the iAnalogsensor YARP device for robot skin
    std::string robot = config.check("robot", yarp::os::Value("icub")).asString();
    std::string iCubSensorPart;

    if (!YarpHelper::getStringFromSearchable(config, "remote_sensor_boards", iCubSensorPart))
    {
        yError() << m_logPrefix << "unable to find remote_sensor_boards into config file.";
        return false;
    }

    yarp::os::Property optionsTactileDevice;

    optionsTactileDevice.put("robot", robot.c_str());
    optionsTactileDevice.put("device", "analogsensorclient");
    optionsTactileDevice.put("local", "/" + robot + "/skin" + "/" + iCubSensorPart + "/in");
    optionsTactileDevice.put("remote", "/" + robot + "/skin" + "/" + iCubSensorPart);

    if (!m_tactileSensorDevice.open(optionsTactileDevice))
    {
        yError() << m_logPrefix << "could not open analogSensorClient object for the robot skin.";
        return false;
    }

    if (!m_tactileSensorDevice.view(m_tactileSensorInterface) || !m_tactileSensorInterface)
    {
        yError() << m_logPrefix << "cannot obtain IAnalogSensor interface for the robot skin";
        return false;
    }

    // get the paramters for the nonlinear mapping of the vibrotactile feedback
    if (!YarpHelper::getVectorFromSearchable(
            config, "absoluteVibrotactileFeedbackNonlinearFunctionParameters", m_fbParams))
    {
        yError() << m_logPrefix
                 << "unable to get absoluteVibrotactileFeedbackNonlinearFunctionParameters"
                 << "from the config file.";
        return false;
    }
    if (m_fbParams.size() != 6)
    {
        yError() << m_logPrefix
                 << "the size of variable `m_fbParams` is not correct, it should be 6, but instead "
                    "it is:"
                 << m_fbParams.size();
        return false;
    }

    // get the percentage for the vibrotactile feedback among the absolute and derivative value
    m_absoluteSkinValuePercentage
        = config.check("absoluteSkinValuePercentage", yarp::os::Value(1.0)).asFloat64();

    m_smoothingGainDerivative
        = config.check("skinDerivativeSmoothingGain", yarp::os::Value(0.9)).asFloat64();

    if (m_absoluteSkinValuePercentage < 0 || m_absoluteSkinValuePercentage > 1.0)
    {
        yError() << m_logPrefix
                 << "absoluteSkinValuePercentage should be between [0, 1], but it is:"
                 << m_absoluteSkinValuePercentage;
        return false;
    }

    // make the vector tactile info for the fingers
    for (const auto& finger : robotFingerNameList)
    {
        FingertipTactileData fingerdata;
        fingerdata.fingerName = finger;
        std::vector<double> tactileInfo; // it should be positive

        if (!YarpHelper::getVectorFromSearchable(
                config, fingerdata.fingerName + "_tactile_info", tactileInfo))
        {
            yError() << m_logPrefix << "unable to get " << fingerdata.fingerName + "_tactile_info"
                     << "from the config file.";
            return false;
        }
        if (tactileInfo.size() != 8)
        {
            yError() << m_logPrefix << "tactile senor indices for "
                     << fingerdata.fingerName + "_tactile_indices"
                     << "size should be 8, but it is not.";
            return false;
        }
        bool check = (std::round(tactileInfo[0]) >= 0) && (std::round(tactileInfo[1]) > 0)
                     && (std::round(tactileInfo[0]) <= std::round(tactileInfo[1]));

        if (!check)
        {
            yError() << m_logPrefix << "tactile senor indices for "
                     << fingerdata.fingerName + "_tactile_indices"
                     << "should be non-negative.";
            return false;
        }
        if (!(tactileInfo[2] >= 0))
        {
            yError() << m_logPrefix << "the contact threshold should be non-negative.";
            return false;
        }

        fingerdata.indexStart = std::round(tactileInfo[0]);
        fingerdata.indexEnd = std::round(tactileInfo[1]);
        fingerdata.noTactileSensors = fingerdata.indexEnd - fingerdata.indexStart + 1;

        fingerdata.contactThresholdValue = tactileInfo[2];
        fingerdata.contactThresholdMultiplier = tactileInfo[3];
        fingerdata.vibrotactileGain = tactileInfo[4];
        fingerdata.contactDerivativeThresholdValue = tactileInfo[5];
        fingerdata.contactDerivativeThresholdMultiplier = tactileInfo[6];
        fingerdata.vibrotactileDerivativeGain = tactileInfo[7];

        fingerdata.rawTactileData.resize(fingerdata.noTactileSensors);
        fingerdata.tactileData.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.biasTactileSensor.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.stdTactileSensor.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.calibratedTactileData.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.previousCalibratedTactileData.resize(fingerdata.noTactileSensors, 0.0);

        fingerdata.tactileDataDerivative.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.biasTactileSensorDerivative.resize(fingerdata.noTactileSensors, 0.0);
        fingerdata.stdTactileSensorDerivative.resize(fingerdata.noTactileSensors, 0.0);

        fingerdata.collectedTactileData.resize(Eigen::NoChange, fingerdata.noTactileSensors);
        fingerdata.collectedTactileDataDerivative.resize(Eigen::NoChange,
                                                         fingerdata.noTactileSensors);

        m_fingersTactileData.push_back(fingerdata);
        m_totalNoTactile += fingerdata.noTactileSensors;
    }
    yInfo() << m_logPrefix << "====== Skin Information ======";
    yInfo() << m_logPrefix << "number of fingers: " << m_noFingers;
    yInfo() << m_logPrefix << "number of tactile sensors: " << m_totalNoTactile;
    yInfo() << m_logPrefix << "tactile working threshold: " << m_tactileWorkingThreshold;
    yInfo() << m_logPrefix << "tactile update threshold: " << m_tactileUpdateThreshold;

    for (const auto& finger : m_fingersTactileData)
        finger.printInfo();

    // Initialize RPC
    std::string rpcPortName = "/robotSkin/rpc";
    if(!m_rpcPort.open(portPrefix + rpcPortName))
    {
        yWarning() << m_logPrefix << "Failed to open" << portPrefix + rpcPortName;
    }
    else
    {
        if (!this->yarp().attachAsServer(m_rpcPort)) {
            yWarning() << m_logPrefix << "Failed to attach" << portPrefix + rpcPortName << "to the RPC service";
        }
    }

    return true;
}

void RobotSkin::updateCalibratedTactileData()
{
    for (auto& finger : m_fingersTactileData)
    {
        // check if the tactile data is updated, otherwise the rate of change of tactile
        // data will stay zero, and will act as a noise.
        // if not updated, remain as the last data.
        for (size_t i = 0; i < finger.noTactileSensors; i++)
        {
            finger.rawTactileData[i]
                = m_fingertipRawTactileFeedbacksStdVector[i + finger.indexStart];

            // crop the data to be sure they are in the range of 0-255
            finger.tactileData[i] = std::max(
                std::min(finger.rawTactileData[i], finger.maxTactileValue), finger.minTactileValue);

            // normalize the data such that:
            // range: [0,1] ; 0: no load, 1: max load
            finger.tactileData[i] = 1.0 - (finger.tactileData[i] / finger.maxTactileValue);

            finger.calibratedTactileData[i] = finger.tactileData[i] - finger.biasTactileSensor[i];

            if (!finger.firstTime)
            {

                double tactileDerivative
                    = (finger.calibratedTactileData[i] - finger.previousCalibratedTactileData[i])
                      / m_samplingTime;

                if (std::abs(tactileDerivative) > m_tactileUpdateThreshold)
                {

                    finger.tactileDataDerivative[i] = tactileDerivative;
                }
            }
            finger.firstTime = false;
            finger.previousCalibratedTactileData[i] = finger.calibratedTactileData[i];
        }
    }
}

bool RobotSkin::collectSkinDataForCalibration()
{

    for (auto& data : m_fingersTactileData)
    {

        Eigen::VectorXd tactileData = CtrlHelper::toEigenVector(data.tactileData);

        if (!CtrlHelper::push_back_row(data.collectedTactileData, tactileData.transpose()))
        {
            yError() << m_logPrefix
                     << "cannot add new axes feedback values to the collected axes data .";
            return false;
        }

        Eigen::VectorXd tactileDataDerivative
            = CtrlHelper::toEigenVector(data.tactileDataDerivative);

        if (!CtrlHelper::push_back_row(data.collectedTactileDataDerivative,
                                       tactileDataDerivative.transpose()))
        {
            yError()
                << m_logPrefix
                << "cannot add new axes feedback values to the collected axes data derivative.";
            return false;
        }
    }

    return true;
}

bool RobotSkin::computeCalibrationParamters()
{
    int counter = 0;
    for (auto& data : m_fingersTactileData)
    {
        bool tactileSenorsWork = false;
        for (size_t i = 0; i < data.noTactileSensors; i++)
        {
            // mean and std of the tactile data
            Eigen::VectorXd vec = data.collectedTactileData.col(i);
            data.biasTactileSensor[i] = vec.mean();
            data.stdTactileSensor[i]
                = std::sqrt(((vec.array() - vec.mean()).square().sum()) / vec.size());

            // mean and std of the tactile data derivative
            Eigen::VectorXd vecDerivative = data.collectedTactileDataDerivative.col(i);
            data.biasTactileSensorDerivative[i] = vecDerivative.mean();
            data.stdTactileSensorDerivative[i]
                = std::sqrt(((vecDerivative.array() - vecDerivative.mean()).square().sum())
                            / vecDerivative.size());

            // if a tactile senors does not work its std is zero
            // normally either all or none of the tactile sensors of a fingertip work
            // so if at least one tactile sensor works, the skin works
            tactileSenorsWork |= (data.stdTactileSensor[i] > m_tactileWorkingThreshold);
        }
        yInfo() << m_logPrefix << data.fingerName << ": mean of tactile sensors"
                << data.biasTactileSensor;
        yInfo() << m_logPrefix << data.fingerName << ": standard deviation of tactile sensors"
                << data.stdTactileSensor;

        yInfo() << m_logPrefix << data.fingerName << ": mean of tactile sensors derivative"
                << data.biasTactileSensorDerivative;
        yInfo() << m_logPrefix << data.fingerName
                << ": standard deviation of tactile sensors derivative"
                << data.stdTactileSensorDerivative;

        if (counter > m_noFingers)
        {
            yError() << m_logPrefix
                     << " the size of m_fingersTactileData is more the number of fingers: "
                     << counter;
            return false;
        }
        m_areTactileSensorsWorking[counter] = tactileSenorsWork;
        counter++;
    }
    yInfo() << m_logPrefix << "are tactile senors working: " << m_areTactileSensorsWorking;
    return true;
}

bool RobotSkin::getFingertipTactileFeedbacks(const size_t fingertipIndex,
                                             std::vector<double>& skinData)
{
    skinData = m_fingersTactileData[fingertipIndex].tactileData;
    return true;
}

bool RobotSkin::getRawTactileFeedbackFromRobot()
{
    if (!(m_tactileSensorInterface->read(m_fingertipRawTactileFeedbacksYarpVector)
          == yarp::dev::IAnalogSensor::AS_OK))
    {
        yWarning() << m_logPrefix << "Unable to get tactile sensor data.";
    }

    CtrlHelper::toStdVector(m_fingertipRawTactileFeedbacksYarpVector,
                            m_fingertipRawTactileFeedbacksStdVector);

    return true;
}

void RobotSkin::updateTactileFeedbacks()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    this->getRawTactileFeedbackFromRobot();

    this->updateCalibratedTactileData();

    this->computeMaxContactStrength();

    this->computeAreFingersInContact();

    this->computeVibrotactileFeedback();
}

void RobotSkin::computeAreFingersInContact()
{
    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_areFingersInContact[i] = m_fingersContactStrength[i]
                                   > m_fingersTactileData[i].contactThreshold();

        // Update the timer only if the same taxel is causing the contact
        if (m_areFingersInContact[i] &&
            m_fingersLastElementInContact[i] == m_fingersTactileData[i].maxTactileFeedbackAbsoluteElement()) {
            m_fingersInContactTimer[i] += m_samplingTime;
        }
        else {
            m_fingersInContactTimer[i] = 0.0;
            m_fingersTactileData[i].isFingerContactEnabled = true;
        }

        // If a taxel is causing the activation for longer then the given time, the sensor bias is increased.
        if (m_fingersInContactTimer[i] > 2.0) { // TODO: move to a parameter
            m_fingersTactileData[i].isFingerContactEnabled = false;
        }

        // Update last finger in contact element fi contact is detected
        m_fingersLastElementInContact[i] = m_areFingersInContact[i] ? m_fingersTactileData[i].maxTactileFeedbackAbsoluteElement() : -1;

        // Check if the finger has been diabled by the timer
        m_areFingersInContact[i] = m_areFingersInContact[i] && m_fingersTactileData[i].isFingerContactEnabled;


        m_areFingersContactChanges[i] = m_areFingersInContact[i]
                                        && (m_fingersContactStrengthDerivate[i]
                                        > m_fingersTactileData[i].contactDerivativeThreshold());
    }
}

void RobotSkin::computeMaxContactStrength()
{

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersContactStrength[i] = m_fingersTactileData[i].maxTactileFeedbackAbsoluteValue();

        m_fingersContactStrengthDerivate[i] = m_fingersTactileData[i].maxTactileFeedbackDerivativeValue();

        m_fingersContactStrengthDerivateSmoothed[i]
            = m_smoothingGainDerivative * m_fingersContactStrengthDerivate[i]
              + (1 - m_smoothingGainDerivative) * m_fingersContactStrengthDerivateSmoothed[i];
    }
}

void RobotSkin::computeVibrotactileFeedback()
{
    for (size_t i = 0; i < m_noFingers; i++)
    {
        double vibrotactileFeedback = m_fingersTactileData[i].vibrotactileGain * (m_areFingersInContact[i] ? m_fingersContactStrength[i] : 0);

        m_fingersVibrotactileAbsoluteFeedback[i]
            = m_fbParams[0] * std::log(m_fbParams[1] * std::pow(vibrotactileFeedback, m_fbParams[2]) + m_fbParams[3])
              + m_fbParams[4] * std::pow(vibrotactileFeedback, m_fbParams[5]);

        double vibrotactileDerivativeFeedback
            = m_fingersTactileData[i].vibrotactileDerivativeGain
              * (m_areFingersContactChanges[i] ? std::abs(m_fingersContactStrengthDerivateSmoothed[i]): 0);
        
        m_fingersVibrotactileDerivativeFeedback[i]
            = m_fbParams[0] * std::log(m_fbParams[1] * std::pow(vibrotactileDerivativeFeedback, m_fbParams[2]) + m_fbParams[3])
              + m_fbParams[4] * std::pow(vibrotactileDerivativeFeedback, m_fbParams[5]);

        // saturate the values between 0 to 100

        m_fingersVibrotactileAbsoluteFeedback[i]
            = std::max(std::min(m_fingersVibrotactileAbsoluteFeedback[i], 100.0), 0.0);

        m_fingersVibrotactileDerivativeFeedback[i]
            = std::max(std::min(m_fingersVibrotactileDerivativeFeedback[i], 100.0), 0.0);

        // compute the total vibrotactile feedback
        m_fingersVibrotactileTotalFeedback[i]
            = m_absoluteSkinValuePercentage * m_fingersVibrotactileAbsoluteFeedback[i]
              + (1.0 - m_absoluteSkinValuePercentage) * m_fingersVibrotactileDerivativeFeedback[i];
    }
}

void RobotSkin::getVibrotactileAbsoluteFeedback(
    std::vector<double>& fingersVibrotactileAbsoluteFeedback)
{
    fingersVibrotactileAbsoluteFeedback = m_fingersVibrotactileAbsoluteFeedback;
}

void RobotSkin::getVibrotactileDerivativeFeedback(
    std::vector<double>& fingersVibrotactileDerivativeFeedback)
{
    fingersVibrotactileDerivativeFeedback = m_fingersVibrotactileDerivativeFeedback;
}

void RobotSkin::getVibrotactileTotalFeedback(std::vector<double>& fingersVibrotactileTotalFeedback)
{
    fingersVibrotactileTotalFeedback = m_fingersVibrotactileTotalFeedback;
}

void RobotSkin::areFingersInContact(std::vector<bool>& areFingersIncontact)
{
    areFingersIncontact = m_areFingersInContact;
}

void RobotSkin::doTactileSensorsWork(std::vector<bool>& tactileSensorsAreWorking)
{
    tactileSensorsAreWorking = m_areTactileSensorsWorking;
}

bool RobotSkin::getSerializedFingertipsTactileFeedbacks(
    std::vector<double>& fingertipsTactileFeedback)
{
    if (fingertipsTactileFeedback.size() != m_totalNoTactile)
    {
        fingertipsTactileFeedback.resize(m_totalNoTactile, 0.0);
    }
    size_t start = 0;
    for (const auto& data : m_fingersTactileData)
    {
        std::copy(data.tactileData.begin(),
                  data.tactileData.end(),
                  fingertipsTactileFeedback.begin() + start);
        start += data.noTactileSensors;
    }

    return true;
}

bool RobotSkin::getSerializedFingertipsCalibratedTactileFeedbacks(
    std::vector<double>& fingertipsTactileFeedback)
{

    if (fingertipsTactileFeedback.size() != m_totalNoTactile)
    {
        fingertipsTactileFeedback.resize(m_totalNoTactile, 0.0);
    }
    size_t start = 0;
    for (const auto& data : m_fingersTactileData)
    {
        std::copy(data.calibratedTactileData.begin(),
                  data.calibratedTactileData.end(),
                  fingertipsTactileFeedback.begin() + start);
        start += data.noTactileSensors;
    }

    return true;
}

bool RobotSkin::getSerializedFingertipsCalibratedTactileDerivativeFeedbacks(
    std::vector<double>& fingertipsTactileDerivativeFeedback)
{

    if (fingertipsTactileDerivativeFeedback.size() != m_totalNoTactile)
    {
        fingertipsTactileDerivativeFeedback.resize(m_totalNoTactile, 0.0);
    }
    size_t start = 0;
    for (const auto& data : m_fingersTactileData)
    {
        std::copy(data.tactileDataDerivative.begin(),
                  data.tactileDataDerivative.end(),
                  fingertipsTactileDerivativeFeedback.begin() + start);
        start += data.noTactileSensors;
    }

    return true;
}

bool RobotSkin::getFingertipsContactStrength(std::vector<double>& fingertipsContactStrength)
{
    fingertipsContactStrength = m_fingersContactStrength;
    return true;
}

bool RobotSkin::getFingertipsContactStrengthDerivative(
    std::vector<double>& fingertipsContactStrengthDerivative)
{
    fingertipsContactStrengthDerivative = m_fingersContactStrengthDerivate;
    return true;
}

const size_t RobotSkin::getNumOfTactileFeedbacks()
{
    return m_totalNoTactile;
}

bool RobotSkin::close()
{
    bool ok = true;
    if (!m_tactileSensorDevice.close())
    {
        yWarning() << m_logPrefix
                   << "Unable to close the tactile sensor analogsensorclient device.";
        ok &= false;
    }
    m_tactileSensorInterface = nullptr;

    m_rpcPort.close();

    yInfo() << m_logPrefix << "closed" << (ok ? "Successfully" : "badly") << ".";

    return ok;
}

const yarp::sig::Vector& RobotSkin::fingerRawTactileFeedbacks() const
{
    return m_fingertipRawTactileFeedbacksYarpVector;
}

void RobotSkin::fingerRawTactileFeedbacks(std::vector<double>& fingertipTactileFeedbacks)
{
    fingertipTactileFeedbacks = m_fingertipRawTactileFeedbacksStdVector;
}

bool RobotSkin::setAbsoluteSkinValuePercentage(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(value<0.0 || value > 1.0 )
    {
        yWarning() << m_logPrefix 
                   << "Requested setAbsoluteSkinValuePercentage with unfeasible value " 
                   << value << "!";
        return false;
    }

    m_absoluteSkinValuePercentage = value;
    return true;
}

bool RobotSkin::setSkinDerivativeSmoothingGain(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(value<0.0 || value > 1.0 )
    {
        yWarning() << m_logPrefix 
                   << "Requested setSkinDerivativeSmoothingGain with unfeasible value " 
                   << value << "!";
        return false;
    }

    m_smoothingGainDerivative = value;
    return true;
}

bool RobotSkin::setContactFeedbackGain(const int32_t finger, const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(value<0.0)
    {
        yWarning() << m_logPrefix 
                   << "Requested setContactFeedbackGain with unfeasible value " 
                   << value << "!";
        return false;
    }
    if (finger >= m_noFingers)
    {
        yWarning() << m_logPrefix 
                   << "Requested setContactFeedbackGain with invalid finger index " 
                   << finger << "!";
        return false;
    }

    m_fingersTactileData[finger].vibrotactileGain = value;
    return true;
}

bool RobotSkin::setContactFeedbackGainAll(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(value<0.0)
    {
        yWarning() << m_logPrefix 
                   << "Requested setContactFeedbackGainAll with unfeasible value " 
                   << value << "!";
        return false;
    }

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersTactileData[i].vibrotactileGain = value;
    }
    
    return true;
}

bool RobotSkin::setDerivativeFeedbackGain(const int32_t finger, const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(value<0.0)
    {
        yWarning() << m_logPrefix 
                   << "Requested setDerivativeFeedbackGain with unfeasible value " 
                   << value << "!";
        return false;
    }
    if (finger >= m_noFingers)
    {
        yWarning() << m_logPrefix 
                   << "Requested setDerivativeFeedbackGain with invalid finger index " 
                   << finger << "!";
        return false;
    }

    m_fingersTactileData[finger].vibrotactileDerivativeGain = value;
    return true;
}

bool RobotSkin::setDerivativeFeedbackGainAll(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(value<0.0)
    {
        yWarning() << m_logPrefix 
                   << "Requested setDerivativeFeedbackGainAll with unfeasible value " 
                   << value << "!";
        return false;
    }

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersTactileData[i].vibrotactileDerivativeGain = value;
    }

    return true;
}

bool RobotSkin::setContactThreshold(const int32_t finger, const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (finger >= m_noFingers)
    {
        yWarning() << m_logPrefix 
                   << "Requested setContactThreshold with invalid finger index " 
                   << finger << "!";
        return false;
    }

    m_fingersTactileData[finger].contactThresholdValue = value;
    return true;
}

bool RobotSkin::setContactThresholdAll(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersTactileData[i].contactThresholdValue = value;
    }

    return true;
}

bool RobotSkin::setContactThresholdMultiplier(const int32_t finger, const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (finger >= m_noFingers)
    {
        yWarning() << m_logPrefix 
                   << "Requested setContactThresholdMultiplier with invalid finger index " 
                   << finger << "!";
        return false;
    }

    m_fingersTactileData[finger].contactThresholdMultiplier = value;
    return true;
}

bool RobotSkin::setContactThresholdMultiplierAll(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersTactileData[i].contactThresholdMultiplier = value;
    }

    return true;
}

bool RobotSkin::setDerivativeThreshold(const int32_t finger, const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (finger >= m_noFingers)
    {
        yWarning() << m_logPrefix 
                   << "Requested setDerivativeThreshold with invalid finger index " 
                   << finger << "!";
        return false;
    }

    m_fingersTactileData[finger].contactDerivativeThresholdValue = value;
    return true;
}

bool RobotSkin::setDerivativeThresholdAll(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersTactileData[i].contactDerivativeThresholdValue = value;
    }

    return true;
}

bool RobotSkin::setDerivativeThresholdMultiplier(const int32_t finger, const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (finger >= m_noFingers)
    {
        yWarning() << m_logPrefix 
                   << "Requested setDerivativeThresholdMultiplier with invalid finger index " 
                   << finger << "!";
        return false;
    }

    m_fingersTactileData[finger].contactDerivativeThresholdMultiplier = value;
    return true;
}

bool RobotSkin::setDerivativeThresholdMultiplierAll(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (size_t i = 0; i < m_noFingers; i++)
    {
        m_fingersTactileData[i].contactDerivativeThresholdMultiplier = value;
    }

    return true;
}
