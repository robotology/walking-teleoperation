/**
 * @file RobotSkin.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef ROBOT_SKIN_HPP
#define ROBOT_SKIN_HPP

// std
#include <algorithm> // std::min_element, std::max_element
#include <iostream>
#include <memory>

// teleoperation
#include <ControlHelper.hpp>
#include <RobotInterface.hpp>

// yarp
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Searchable.h>

// rpc service
#include <thrift/RobotSkinService.h>

namespace HapticGlove
{
class RobotSkin;
struct FingertipTactileData;
} // namespace HapticGlove

/**
 * FingertipTactileData structure useful to manage the fingertip skin data.
 */
struct HapticGlove::FingertipTactileData
{
    const double maxTactileValue = 255.0;
    const double minTactileValue = 0.0;
    const double noLoadValue = 240.0;

    std::string fingerName;
    size_t indexStart;
    size_t indexEnd;
    size_t noTactileSensors;
    bool firstTime = true;

    double contactThresholdValue = 5.0; /// default value
    double contactThresholdMultiplier = 1.0; /// default value
    double contactDerivativeThresholdValue = 3.0; /// default value
    double contactDerivativeThresholdMultiplier = 1.0; /// default value

    double vibrotactileGain = 1.0; /// default value
    double vibrotactileDerivativeGain = 1.0; /// default value

    std::vector<double>
        rawTactileData; /// range: 0-256; value 240 shows no load, and 0 shows max load

    std::vector<double> tactileData; /// range: 0-1; 0 shows no load, and 1 shows max load
    std::vector<double> tactileDataDerivative; /// shows the derivative of the tactile data

    std::vector<double> calibratedTactileData; /// range: almost 0-1: 0: no load, 1 max load
    std::vector<double> previousCalibratedTactileData; /// range: almost 0-1: 0: no load, 1 max load

    std::vector<double> biasTactileSensor; /// mean of the tactile sensors when not touched
    std::vector<double>
        biasTactileSensorDerivative; /// mean of the tactile sensors derivative when not touched

    std::vector<double>
        stdTactileSensor; /// standard deviation (std) of the tactile sensors when not touched

    std::vector<double> stdTactileSensorDerivative; /// standard deviation (std) of the tactile
                                                    /// sensors derivative when not touched

    CtrlHelper::Eigen_Mat collectedTactileData; /**< The logged data to find the bias and standard
                            deviation (std) of tactile sensors;
                            - dimension <o, t>:
                            - o: number of observations (logged data),
                            - m: number of tactile sensors*/

    CtrlHelper::Eigen_Mat collectedTactileDataDerivative; /**< The logged data to find the bias and
                            standard deviation (std) of tactile sensors;
                            - dimension <o, t>:
                            - o: number of observations (logged data),
                            - m: number of tactile sensors*/

    bool isFingerContactEnabled = true;

    double maxTactileFeedbackAbsoluteValue()
    {
        return *std::max_element(calibratedTactileData.begin(), calibratedTactileData.end());
    }
    size_t maxTactileFeedbackAbsoluteElement()
    {
        return std::distance(
            calibratedTactileData.begin(),
            std::max_element(calibratedTactileData.begin(), calibratedTactileData.end()));
    }

    size_t maxTactileFeedbackDerivativeElement()
    {
        return std::distance(
            tactileDataDerivative.begin(),
            std::max_element(tactileDataDerivative.begin(), tactileDataDerivative.end()));
    }

    double maxTactileFeedbackDerivativeValue()
    {
        // we check for the derivative of the tactile sensor with the highest calibrated absolute
        // value, since we believe the value of th that tactile sensor is more important. the
        // hypothesis should be checked.
        //        return tactileDataDerivative[this->maxTactileFeedbackAbsoluteElement()];

        return *std::max_element(tactileDataDerivative.begin(), tactileDataDerivative.end());
    }

    double contactThreshold()
    {
        return contactThresholdValue 
               + contactThresholdMultiplier * stdTactileSensor[this->maxTactileFeedbackAbsoluteElement()];
    }

    double contactDerivativeThreshold()
    {
        return contactDerivativeThresholdValue
               + contactDerivativeThresholdMultiplier * stdTactileSensorDerivative[this->maxTactileFeedbackDerivativeElement()];
    }

    void printInfo() const
    {
        std::cout << "==================" << std::endl;
        std::cout << "finger name: " << fingerName << std::endl;
        std::cout << "starting index: " << indexStart << std::endl;
        std::cout << "ending index: " << indexEnd << std::endl;
        std::cout << "number of tactile sensor: " << noTactileSensors << std::endl;
        std::cout << "max tactile threshold: " << maxTactileValue << std::endl;
        std::cout << "min tactile threshold: " << minTactileValue << std::endl;
        std::cout << "no load tactile threshold: " << noLoadValue << std::endl;
        std::cout << "contact threshold: " << contactThresholdValue << std::endl;
        std::cout << "contact threshold multiplier: " << contactThresholdMultiplier << std::endl;
        std::cout << "contact derivative threshold: " << contactDerivativeThresholdValue
                  << std::endl;
        std::cout << "contact derivative threshold multiplier: " << contactDerivativeThresholdMultiplier
                  << std::endl;
        std::cout << "vibrotactile gain: " << vibrotactileGain << std::endl;
        std::cout << "==================" << std::endl;
    }
};

/**
 * RobotSkin Class useful to manage the fingertip skin data.
 */
class HapticGlove::RobotSkin : RobotSkinService
{
private:
    std::string m_logPrefix;

    bool m_rightHand;
    size_t m_noFingers;
    size_t m_totalNoTactile;
    double m_samplingTime;
    std::vector<FingertipTactileData> m_fingersTactileData;

    std::vector<bool> m_areTactileSensorsWorking;
    std::vector<bool> m_areFingersInContact;
    std::vector<double> m_fingersInContactTimer;
    std::vector<int> m_fingersLastElementInContact;
    std::vector<bool> m_areFingersContactChanges;

    std::vector<double> m_fingersVibrotactileAbsoluteFeedback;
    std::vector<double> m_fingersVibrotactileDerivativeFeedback;
    std::vector<double> m_fingersVibrotactileTotalFeedback;

    std::vector<double> m_fingersContactStrength;
    std::vector<double> m_fingersContactStrengthDerivate;
    std::vector<double> m_fingersContactStrengthDerivateSmoothed;
    double m_smoothingGainDerivative; // exponential filter smoothing gain

    double m_tactileWorkingThreshold;
    double m_tactileUpdateThreshold; // this is a threshold to check if the sensor data is updated,
                                     // this is added because tactile senor update rates are lower
                                     // than the module update rate.

    yarp::sig::Vector
        m_fingertipRawTactileFeedbacksYarpVector; /**< fingertip raw tactile feedbacks, `0`
                                        means high pressure, `255` means low pressure */
    std::vector<double>
        m_fingertipRawTactileFeedbacksStdVector; /**< fingertip raw tactile feedbacks, `0`
                                        means high pressure, `255` means low pressure */

    yarp::dev::PolyDriver m_tactileSensorDevice; /**< Analog device for the skin. */

    yarp::dev::IAnalogSensor* m_tactileSensorInterface{
        nullptr}; /**< skin ananlog sensor interface */

    std::vector<double>
        m_fbParams; /**< # absolute vibrotactile feedback nonlinear function parameters; reference
                     to
                     https://github.com/ami-iit/element_retargeting-from-human/issues/182#issuecomment-1000472012
                   */

    double m_absoluteSkinValuePercentage; /* percentage dedicated to absolute skin data  (among
                                           * absolute and derivative)for providing the vibrotactile
                                           * feedback, the value is between [0, 1]
                                           */
    // mutex
    std::mutex m_mutex;

    // RPC port
    yarp::os::Port m_rpcPort;
    
    void updateCalibratedTactileData();

    void computeVibrotactileFeedback();

    void computeMaxContactStrength();

    void computeAreFingersInContact();

    bool getRawTactileFeedbackFromRobot();

public:
    RobotSkin();
    /**
     * Configure the object.
     * @param config reference to a resource finder object.
     * @param name name of the robot
     * @return true in case of success and false otherwise.
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    void updateTactileFeedbacks();

    bool computeCalibrationParamters();

    bool collectSkinDataForCalibration();

    bool getFingertipTactileFeedbacks(const size_t fingertipIndex, std::vector<double>& skinData);

    bool getSerializedFingertipsTactileFeedbacks(std::vector<double>& fingertipsTactileFeedback);

    bool getSerializedFingertipsCalibratedTactileFeedbacks(
        std::vector<double>& fingertipsTactileFeedback);

    bool getSerializedFingertipsCalibratedTactileDerivativeFeedbacks(
        std::vector<double>& fingertipsTactileDerivativeFeedback);

    bool getFingertipsContactStrength(std::vector<double>& fingertipsContactStrength);

    bool getFingertipsContactStrengthDerivative(
        std::vector<double>& fingertipsContactStrengthDerivative);

    void areFingersInContact(std::vector<bool>& areFingersIncontact);

    void getVibrotactileAbsoluteFeedback(std::vector<double>& fingersVibrotactileAbsoluteFeedback);

    void
    getVibrotactileDerivativeFeedback(std::vector<double>& fingersVibrotactileDerivativeFeedback);

    void getVibrotactileTotalFeedback(std::vector<double>& fingersVibrotactileTotalFeedback);

    const size_t getNumOfTactileFeedbacks();

    void doTactileSensorsWork(std::vector<bool>& tactileSensorsAreWorking);

    /**
     * Get the fingertip calibrated tactile feedbacks
     * @return fingertip calibrated tactile feedbacks
     */
    const yarp::sig::Vector& fingerRawTactileFeedbacks() const;

    /**
     * Get the fingertip calibrated tactile feedbacks
     * @param fingertipTactileFeedbacks the tactile feedbacks of all the links
     */
    void fingerRawTactileFeedbacks(std::vector<double>& fingertipTactileFeedbacks);

    virtual bool setAbsoluteSkinValuePercentage(const double value) override;

    virtual bool setSkinDerivativeSmoothingGain(const double value) override;

    virtual bool setContactFeedbackGain(const int32_t finger, const double value) override;

    virtual bool setContactFeedbackGainAll(const double value) override;

    virtual bool setDerivativeFeedbackGain(const int32_t finger, const double value) override;

    virtual bool setDerivativeFeedbackGainAll(const double value) override;

    virtual bool setContactThreshold(const int32_t finger, const double value) override;

    virtual bool setContactThresholdAll(const double value) override;

    virtual bool setContactThresholdMultiplier(const int32_t finger, const double value) override;

    virtual bool setContactThresholdMultiplierAll(const double value) override;

    virtual bool setDerivativeThreshold(const int32_t finger, const double value) override;

    virtual bool setDerivativeThresholdAll(const double value) override;

    virtual bool setDerivativeThresholdMultiplier(const int32_t finger, const double value) override;

    virtual bool setDerivativeThresholdMultiplierAll(const double value) override;

    bool close();
};

#endif // ROBOT_SKIN_HPP
