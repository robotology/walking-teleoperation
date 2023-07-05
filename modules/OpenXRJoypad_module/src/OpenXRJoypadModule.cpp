/**
 * @file JoypadFingersModule.cpp
 * @copyright 2023 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <functional>
#include <iterator>
#include <thread>
#include <tuple>

// YARP
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <iCub/ctrl/pids.h>

#include <OpenXRJoypadModule.hpp>
#include <Utils.hpp>

class RobotControlHelper
{
    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */

    int m_actuatedDOFs; /**< Number of the actuated DoF */

    std::vector<std::string>
        m_axesList; /**< Vector containing the name of the controlled joints. */

    yarp::dev::IEncodersTimed* m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* m_positionDirectInterface{nullptr}; /**< Direct position control
                                                                       interface. */
    yarp::dev::IVelocityControl* m_velocityInterface{nullptr}; /**< Velocity control interface. */
    yarp::dev::IControlMode* m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits* m_limitsInterface{nullptr}; /**< Encorders interface. */

    yarp::sig::Vector m_desiredJointValue; /**< Desired joint value [deg or deg/s]. */
    yarp::sig::Vector m_positionFeedbackInDegrees; /**< Joint position [deg]. */
    yarp::sig::Vector m_positionFeedbackInRadians; /**< Joint position [rad]. */

    yarp::conf::vocab32_t m_controlMode; /**< Used control mode. */

    /**
     * Switch to control mode
     * @param controlMode is the specific control mode
     * @return true / false in case of success / failure
     */
    bool switchToControlMode(const int& controlMode);

    /**
     * Set the desired joint position (position direct mode)
     * desiredPosition desired joint position in radiant
     * @return true / false in case of success / failure
     */
    bool setDirectPositionReferences(const yarp::sig::Vector& desiredPositions);

    /**
     * Set the desired joint velocity
     * @param desiredVelocity desired joint velocity in radiant/s
     * @return true / false in case of success / failure
     */
    bool setVelocityReferences(const yarp::sig::Vector& desiredVelocity);

public:
    /**
     * Configure the helper
     * @param config confifuration options
     * @param name name of the robot
     * @param isMandatory if true the helper will return an error if there is a
     * problem in the configuration phase
     * @return true / false in case of success / failure
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name);

    /**
     * Set the desired joint reference (position or velocity)
     * @param desiredValue desired joint velocity or position (radiant or radiant/s)
     * @return true / false in case of success / failure
     */
    bool setJointReference(const yarp::sig::Vector& desiredValue);

    /**
     * Check if the velocity control is used
     * @return true if the velocity control is used
     */
    bool isVelocityControlUsed();

    /**
     * Get feedback from the robot
     * @return true / false in case of success / failure
     */
    bool getFeedback();

    /**
     * Get the joint limits
     * @param limits matrix containing the joint limits in radian
     * @return true / false in case of success / failure
     */
    bool getLimits(yarp::sig::Matrix& limits);

    /**
     * Get the joint encoders value
     * @return the joint encoder value
     */
    const yarp::sig::Vector& jointEncoders() const;

    /**
     * Get the number of degree of freedom
     * @return the number of actuated DoF
     */
    int getDoFs();

    /**
     * Close the helper
     */
    void close();
};

bool RobotControlHelper::configure(const yarp::os::Searchable& config,
                                   const std::string& name)
{

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value* iCubPartsYarp;
    if (!config.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find remote_control_boards into "
                    "config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a vector of "
                    "strings.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    yarp::os::Value* axesListYarp;
    if (!config.check("joints_list", axesListYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find joints_list into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }

    options.put("device", "remotecontrolboardremapper");
    YarpHelper::addVectorOfStringToProperty(options, "axesNames", m_axesList);

    // prepare the remotecontrolboards
    yarp::os::Bottle remoteControlBoards;
    remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (auto& iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    options.put("remoteControlBoards", remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");

    m_actuatedDOFs = m_axesList.size();

    bool useVelocity = config.check("useVelocity", yarp::os::Value(false)).asBool();
    m_controlMode = useVelocity ? VOCAB_CM_VELOCITY : VOCAB_CM_POSITION_DIRECT;

    // open the device
    if (!m_robotDevice.open(options))
    {
        yError() << "[RobotControlHelper::configure] Could not open remotecontrolboardremapper "
                    "object.";
    }

    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IEncoders interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IVelocityInterface interface";
        return false;
    }

    if (!m_robotDevice.view(m_limitsInterface) || !m_limitsInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IControlMode interface";
        return false;
    }

    m_desiredJointValue.resize(m_actuatedDOFs);
    m_positionFeedbackInDegrees.resize(m_actuatedDOFs);
    m_positionFeedbackInRadians.resize(m_actuatedDOFs);

    // check if the robot is alive
    bool okPosition = false;
    for (int i = 0; i < 10 && !okPosition; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());

        if (!okPosition)
            yarp::os::Time::delay(0.1);
    }
    if (!okPosition)
    {
        yError() << "[RobotControlHelper::configure] Unable to read encoders (position).";
        return false;
    }

    if (!switchToControlMode(m_controlMode))
    {
        yError() << "[RobotControlHelper::configure] Unable to switch the control mode";
        return false;
    }
    return true;
}

bool RobotControlHelper::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if (!m_controlModeInterface)
    {
        //yError() << "[RobotControlHelper::switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if (!m_controlModeInterface->setControlModes(controlModes.data()))
    {
        yError()
            << "[RobotControlHelper::switchToControlMode] Error while setting the controlMode.";
    }
    return true;
}

bool RobotControlHelper::setDirectPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionDirectInterface == nullptr)
    {
        yError()
            << "[RobotControlHelper::setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlHelper::setDirectPositionReferences] Dimension mismatch between "
                    "desired position vector and the number of controlled joints.";
        return false;
    }

    // convert radiant to degree
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredPosition(i));

    // set desired position
    m_positionDirectInterface->setPositions(m_desiredJointValue.data());

    return true;
}

bool RobotControlHelper::setVelocityReferences(const yarp::sig::Vector& desiredVelocity)
{
    if (m_velocityInterface == nullptr)
    {
        //yError() << "[RobotControlHelper::setVelocityReferences] Velocity I/F not ready.";
        return false;
    }

    if (desiredVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlHelper::setVelocityReferences] Dimension mismatch between "
                    "desired velocity vector and the number of controlled joints.";
        return false;
    }

    // convert radiant/s  to degree/s
    for (int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredVelocity(i));

    // since the velocity interface use a minimum jerk trajectory a very high acceleration is set in
    // order to use it as velocity "direct" interface
    yarp::sig::Vector dummy(m_actuatedDOFs, std::numeric_limits<double>::max());
    m_velocityInterface->setRefAccelerations(dummy.data());
    m_velocityInterface->velocityMove(m_desiredJointValue.data());
    return true;
}

bool RobotControlHelper::getFeedback()
{
    m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());

    for (unsigned j = 0; j < m_actuatedDOFs; ++j)
        m_positionFeedbackInRadians(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));

    return true;
}


const yarp::sig::Vector& RobotControlHelper::jointEncoders() const
{
    return m_positionFeedbackInRadians;
}

void RobotControlHelper::close()
{
    if (!switchToControlMode(VOCAB_CM_POSITION))
        yError() << "[RobotControlHelper::close] Unable to switch in position control.";

    if (!m_robotDevice.close())
        yError() << "[RobotControlHelper::close] Unable to close the device.";
}

int RobotControlHelper::getDoFs()
{
    return m_actuatedDOFs;
}

bool RobotControlHelper::getLimits(yarp::sig::Matrix& limits)
{
    if (!getFeedback())
    {
        yError() << "[RobotControlHelper::getLimits] Unable to get the feedback from the robot";
        return false;
    }

    // resize matrix
    limits.resize(m_actuatedDOFs, 2);

    double maxLimitInDegree, minLimitInDegree;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        // get position limits
        if (!m_limitsInterface->getLimits(i, &minLimitInDegree, &maxLimitInDegree))
        {

            limits(i, 0) = m_positionFeedbackInRadians(i);
            limits(i, 1) = m_positionFeedbackInRadians(i);
            yWarning()
                    << "[RobotControlHelper::getLimits] Unable get " << m_axesList[i]
                       << " joint limits. The current joint value is used as lower and upper limits.";

        }
        else
        {
            limits(i, 0) = iDynTree::deg2rad(minLimitInDegree);
            limits(i, 1) = iDynTree::deg2rad(maxLimitInDegree);
        }
    }
    return true;
}

bool RobotControlHelper::setJointReference(const yarp::sig::Vector& desiredValue)
{
    switch (m_controlMode)
    {
    case VOCAB_CM_POSITION_DIRECT:
        if (!setDirectPositionReferences(desiredValue))
        {
            yError() << "[RobotControlHelper::setJointReference] Unable to set the desired joint "
                        "position";
            return false;
        }
        break;

    case VOCAB_CM_VELOCITY:
        if (!setVelocityReferences(desiredValue))
        {
            //yError() << "[RobotControlHelper::setJointReference] Unable to set the desired joint "
            //            "velocity";
            return false;
        }
        break;

    default:
        yError() << "[RobotControlHelper::setJointReference] Unknown control mode.";
        return false;
    }

    return true;
}

bool RobotControlHelper::isVelocityControlUsed()
{
    return m_controlMode == VOCAB_CM_VELOCITY;
}

class FingersRetargeting
{
private:
    yarp::sig::Vector m_fingersScaling; /**< It contains the finger velocity scaling. */
    std::unique_ptr<iCub::ctrl::Integrator> m_fingerIntegrator{nullptr}; /**< Velocity integrator */
    std::unique_ptr<RobotControlHelper> m_controlHelper; /**< Controller helper */
    yarp::sig::Vector m_desiredJointValue; /** Desired joint value in radiant or radiant/s  */

public:
    /**
     * Configure the object.
     * @param config reference to a resource finder object.
     * @param name name of the robot
     * @return true in case of success and false otherwise.
     */
    virtual bool configure(const yarp::os::Searchable& config, const std::string& name);

    /**
     * Set the fingers velocities
     * @param fingersVelocity value from -1 to 1
     * @return true in case of success and false otherwise.
     */
    bool setFingersVelocity(const double& fingersVelocity);

    /**
     * Move the robot part
     * @return true in case of success and false otherwise.
     */
    virtual bool move();

};

bool FingersRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    m_controlHelper = std::make_unique<RobotControlHelper>();
    if (!m_controlHelper->configure(config, name))
    {
        yError() << "[FingersRetargeting::configure] Unable to configure the control helper";
        return false;
    }

    int fingersJoints = m_controlHelper->getDoFs();

    double samplingTime;
    if (!YarpHelper::getDoubleFromSearchable(config, "samplingTime", samplingTime))
    {
        yError() << "[FingersRetargeting::configure] Unable to find the sampling time";
        return false;
    }

    m_fingersScaling.resize(fingersJoints);
    if (!YarpHelper::getYarpVectorFromSearchable(config, "fingersScaling", m_fingersScaling))
    {
        yError() << "[FingersRetargeting::configure] Initialization failed while reading "
                    "fingersScaling vector.";
        return false;
    }

    m_desiredJointValue.resize(fingersJoints);
    yarp::sig::Vector buff(fingersJoints, 0.0);
    yarp::sig::Matrix limits(fingersJoints, 2);
    if (!m_controlHelper->getLimits(limits))
    {
        yError() << "[FingersRetargeting::configure] Unable to get the joint limits.";
        return false;
    }
    m_fingerIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buff, limits);

    return true;
}

bool FingersRetargeting::setFingersVelocity(const double& fingersVelocity)
{
    if (m_fingerIntegrator == nullptr)
    {
        //yError() << "[FingersRetargeting::setFingersVelocity] The integrator is not initialize "
        //            "please call configure() method";
        return false;
    }

    if (m_controlHelper->isVelocityControlUsed())
        m_desiredJointValue = fingersVelocity * m_fingersScaling;
    else
        m_desiredJointValue = m_fingerIntegrator->integrate(fingersVelocity * m_fingersScaling);
    return true;
}

bool FingersRetargeting::move()
{
    return m_controlHelper->setJointReference(m_desiredJointValue);
}

auto getPosition(const yarp::sig::Matrix& m)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data())
        .topRightCorner<3, 1>();
}


struct OpenXRJoypadModule::Impl
{
    struct JoypadParameters
    {
        double deadzone; /**< Joypad deadzone */
        double fullscale; /**< Joypad fullscale */
        double x; /**< x value */
        double y; /**< y value */
        double z; /**< z value */

        std::vector<int> startWalkingButtonsMap;
        std::vector<int> prepareWalkingButtonsMap;
        std::vector<int> stopWalkingButtonsMap;
        std::vector<int> leftFingersSqueezeButtonsMap;
        std::vector<int> leftFingersReleaseButtonsMap;
        std::vector<int> rightFingersSqueezeButtonsMap;
        std::vector<int> rightFingersReleaseButtonsMap;

        int xJoypadIndex; /**< Mapping of the axis related to x coordinate */
        int yJoypadIndex; /**< Mapping of the axis related to y coordinate */
        int zJoypadIndex; /**< Mapping of the axis related to z coordinate */
        std::vector<int> joypadLeftButtonsMap;
        std::vector<int> joypadRightButtonsMap;

        int fingersVelocityLeftIndex; /**< Index of the trigger used for squeezing the left hand */
        int fingersVelocityRightIndex; /**< Index of the trigger used for
                                           squeezing the right hand */
    };
    JoypadParameters joypadParameters;

    enum class OpenXRFSM
    {
        Configured,
        Running,
        InPreparation
    };
    OpenXRFSM state; /**< State of the OpenXRFSM */

    double dT;

    std::unique_ptr<FingersRetargeting> leftHandFingers; /**< Pointer to the left
                                                              finger retargeting object. */
    std::unique_ptr<FingersRetargeting> rightHandFingers; /**< Pointer to the right
                                                               finger retargeting object. */

    yarp::os::RpcClient rpcWalkingClient; /**< Rpc client used for sending command to the walking
                                               controller */
    yarp::os::BufferedPort<yarp::sig::Vector> robotGoalPort; /**< Port used to specify the desired goal position. */


    // transform server
    yarp::dev::PolyDriver transformClientDevice; /**< Transform client. */
    yarp::dev::IFrameTransform* frameTransformInterface{nullptr}; /**< Frame transform
                                                                       interface. */
    yarp::dev::PolyDriver joypadDevice; /**< Joypad polydriver. */
    yarp::dev::IJoypadController* joypadControllerInterface{nullptr}; /**< joypad interface. */

    std::string headFrameName; /**< Name of the head frame used in the transform server */
    std::string leftHandFrameName; /**< Name of the left hand frame used in the transform server */
    std::string rightHandFrameName; /**< Name of the right hand
                                       frame used in the transform server */

    bool leftAndRightSwapped{false};
    std::vector<int> buttonsState;
    unsigned int buttonCount = 0;

    bool isButtonStateEqualToMask(const std::vector<int>& mask) const
    {
        return (mask.size() <= this->buttonsState.size()
                && std::equal(mask.begin(),
                              mask.end(),
                              this->buttonsState.begin(),
                              [](const auto& a, const auto& b) {
                                  if (a < 0)
                                      return true;
                                  return a == b;
                              }));
    }


    bool configureTranformClient(const yarp::os::Searchable& config,
                                 const std::string& applicationName)
    {
       std::string transformServerRemote = config.check("transform_server_remote", yarp::os::Value("/transformServer")).asString();
        yarp::os::Property options;
        options.put("device", "transformClient");
        options.put("remote", transformServerRemote);
        options.put("local", "/" + applicationName + "/transformClient");

        if (!this->transformClientDevice.open(options))
        {
            yError() << "[JoypadFingersModule::configureTranformClient] Unable "
                        "to open transformClient device";
            return false;
        }

        // obtain the interface
        if (!this->transformClientDevice.view(this->frameTransformInterface)
            || this->frameTransformInterface == nullptr)
        {
            yError() << "[JoypadFingersModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, "head_frame_name", this->headFrameName))
        {

            yError() << "[JoypadFingersModule::configureTranformClient] Seems that the head "
                        "orientation is not streamed through the transform server.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, //
                                                 "left_hand_frame_name",
                                                 this->leftHandFrameName))
        {
            yError() << "[JoypadFingersModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, //
                                                 "right_hand_frame_name",
                                                 this->rightHandFrameName))
        {
            yError() << "[JoypadFingersModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        return true;
    }

    bool configureJoypad(const yarp::os::Searchable& config, const std::string& name)
    {
        std::string joypadDeviceRemote;
        if (!YarpHelper::getStringFromSearchable(config,
                                                 "joypad_device_remote", //
                                                 joypadDeviceRemote))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter joypad_device_remote";
            return false;
        }

        yarp::os::Property options;
        options.put("device", "JoypadControlClient");
        options.put("remote", joypadDeviceRemote);
        options.put("local", "/" + name + "/joypadControlClient");

        if (!this->joypadDevice.open(options))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to open the polydriver.";
            return false;
        }

        // get the interface
        if (!this->joypadDevice.view(this->joypadControllerInterface)
            || this->joypadControllerInterface == nullptr)
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to attach JoypadController"
                        " interface to the PolyDriver object";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config,
                                                 "deadzone", //
                                                 this->joypadParameters.deadzone))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter deadzone";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config, //
                                                 "fullscale",
                                                 this->joypadParameters.fullscale))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter fullscale";
            return false;
        }

        int leftXIndex;
        int leftYIndex;
        int rightXIndex;
        int rightYIndex;

        if (!YarpHelper::getIntFromSearchable(config, //
                                              "left_x_index",
                                              leftXIndex))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter left_x_index";
            return false;
        }

        if (!YarpHelper::getIntFromSearchable(config, //
                                              "left_y_index",
                                              leftYIndex))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter left_y_index";
            return false;
        }

        if (!YarpHelper::getIntFromSearchable(config, //
                                              "right_x_index",
                                              rightXIndex))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter right_x_index";
            return false;
        }

        if (!YarpHelper::getIntFromSearchable(config, //
                                              "right_y_index",
                                              rightYIndex))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter right_y_index";
            return false;
        }

        // this vector of pointers of maps will simplify some checks later on
        std::vector<std::vector<int>*> buttonsMap;
        std::vector<std::vector<int>*> stateMachineButtonsMap;

        // set the index of the axis according to the OVRheadset yarp device
        //  The order of the buttons are here: https://github.com/ami-iit/yarp-device-openxrheadset/blob/b560d603bba8e50415be839d0e22b51219abbda8/src/devices/openxrheadset/OpenXrInterface.cpp#L651-L661

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "start_walking_buttons_map",
                                                    this->joypadParameters.startWalkingButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "start_walking_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.startWalkingButtonsMap);
        stateMachineButtonsMap.push_back(&this->joypadParameters.startWalkingButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "prepare_walking_buttons_map",
                                                    this->joypadParameters.prepareWalkingButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "prepare_walking_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.prepareWalkingButtonsMap);
        stateMachineButtonsMap.push_back(&this->joypadParameters.prepareWalkingButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "left_fingers_squeeze_buttons_map",
                                                    this->joypadParameters.leftFingersSqueezeButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "left_fingers_squeeze_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.leftFingersSqueezeButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "left_fingers_release_buttons_map",
                                                    this->joypadParameters.leftFingersReleaseButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "left_fingers_release_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.leftFingersReleaseButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "right_fingers_squeeze_buttons_map",
                                                    this->joypadParameters.rightFingersSqueezeButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "right_fingers_squeeze_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.rightFingersSqueezeButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "right_fingers_release_buttons_map",
                                                    this->joypadParameters.rightFingersReleaseButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "right_fingers_release_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.rightFingersReleaseButtonsMap);

        // check if the size of all the maps are the same. If not there is a mistake in the
        // configuration file.
        const auto size = this->joypadParameters.rightFingersReleaseButtonsMap.size();
        for (const auto map : buttonsMap)
        {
            if (size != map->size())
            {
                yError()
                    << "[JoypadFingersModule::configureJoypad] Mismatch in the size of the buttons map. "
                       "Please check the configuration file";
                return false;
            }
        }

        std::vector<int> leftWalkingButtonsMap;
        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "left_walking_buttons_map",
                                                    leftWalkingButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "left_walking_buttons_map";
            return false;
        }
        buttonsMap.push_back(&leftWalkingButtonsMap);

        std::vector<int> rightWalkingButtonsMap;
        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "right_walking_buttons_map",
                                                    rightWalkingButtonsMap))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "right_walking_buttons_map";
            return false;
        }
        buttonsMap.push_back(&rightWalkingButtonsMap);


        auto isEqual = [](const auto& v1, const auto& v2) {
            return (v1->size() == v2->size() && std::equal(v1->begin(), v1->end(), v2->begin()));
        };

        for (auto it = stateMachineButtonsMap.cbegin(); it != stateMachineButtonsMap.cend();
             std::advance(it, 1))
        {
            for (auto otherIt = std::next(it, 1); otherIt != stateMachineButtonsMap.cend();
                 std::advance(otherIt, 1))
            {
                if (isEqual(*it, *otherIt))
                {
                    yError() << "[JoypadFingersModule::configureJoypad] State machine maps cannot must be "
                                "different";
                    return false;
                }
            }
        }

        // the stop button mask is the end of the prepare and start mask
        this->joypadParameters.stopWalkingButtonsMap.resize(
            this->joypadParameters.startWalkingButtonsMap.size());
        for (int i = 0; i < this->joypadParameters.startWalkingButtonsMap.size(); i++)
        {
            this->joypadParameters.stopWalkingButtonsMap[i]
                = (this->joypadParameters.startWalkingButtonsMap[i] > 0
                   || this->joypadParameters.prepareWalkingButtonsMap[i] > 0)
                      ? 1
                      : 0;
        }

        // in the vive (yarp) we have the following axis
        // [vive_left_trigger, vive_right_trigger, vive_left_trackpad_x, vive_left_trackpad_y,
        // vive_right_trackpad_x, vive_right_trackpad_y]

        this->joypadParameters.fingersVelocityLeftIndex = !this->leftAndRightSwapped ? 0 : 1; //TODO CONF FILE
        this->joypadParameters.fingersVelocityRightIndex = !this->leftAndRightSwapped ? 1 : 0;

        this->joypadParameters.xJoypadIndex
            = this->leftAndRightSwapped ? rightXIndex : leftXIndex;
        this->joypadParameters.yJoypadIndex
            = this->leftAndRightSwapped ? rightYIndex : leftYIndex;
        this->joypadParameters.zJoypadIndex
            = this->leftAndRightSwapped ? leftYIndex : rightYIndex;
        this->joypadParameters.joypadLeftButtonsMap = this->leftAndRightSwapped ? rightWalkingButtonsMap : leftWalkingButtonsMap;
        this->joypadParameters.joypadRightButtonsMap = this->leftAndRightSwapped ? leftWalkingButtonsMap : rightWalkingButtonsMap;

        // if swapped we have to swap all the maps
        if (this->leftAndRightSwapped)
        {
            std::swap(this->joypadParameters.rightFingersReleaseButtonsMap,
                      this->joypadParameters.leftFingersReleaseButtonsMap);
            std::swap(this->joypadParameters.rightFingersSqueezeButtonsMap,
                      this->joypadParameters.leftFingersSqueezeButtonsMap);
            std::swap(this->joypadParameters.startWalkingButtonsMap,
                      this->joypadParameters.prepareWalkingButtonsMap);
        }

        return true;
    }

    bool setLeftAndRightSwappedFlag()
    {

        auto readTransforms = [this](yarp::sig::Matrix& headOpenXR_T_leftHandOpenXR,
                                     yarp::sig::Matrix& headOpenXR_T_rightHandOpenXR) {
            bool ok = true;

            ok = ok && this->frameTransformInterface->frameExists(this->headFrameName);
            ok = ok && this->frameTransformInterface->frameExists(this->leftHandFrameName);
            ok = ok && this->frameTransformInterface->frameExists(this->rightHandFrameName);


            ok = ok && this->frameTransformInterface->getTransform(this->rightHandFrameName, //
                                                                   this->headFrameName,
                                                                   headOpenXR_T_rightHandOpenXR);

            ok = ok && this->frameTransformInterface->getTransform(this->leftHandFrameName, //
                                                                   this->headFrameName,
                                                                   headOpenXR_T_leftHandOpenXR);

            return ok;
        };

        yarp::sig::Matrix headOpenXR_T_expectedLeftHandOpenXR;
        yarp::sig::Matrix headOpenXR_T_expectedRightHandOpenXR;
        headOpenXR_T_expectedLeftHandOpenXR.resize(4, 4);
        headOpenXR_T_expectedRightHandOpenXR.resize(4, 4);

        // try to read the transform
        std::size_t counter = 0;
        constexpr unsigned int maxAttempt = 1000;
        while (!readTransforms(headOpenXR_T_expectedLeftHandOpenXR, headOpenXR_T_expectedRightHandOpenXR))
        {
            if (++counter == maxAttempt)
            {
                yError() << "[JoypadFingersModule::setLeftAndRightJoypad] Unable to read the "
                            "transform client.";
                return false;
            }

            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        auto getPosition = [](const yarp::sig::Matrix& m)
        {
            return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data())
                .topRightCorner<3, 1>();
        };

        // check if the left joypad is on the left and the right is on the right. If not we have to
        // swap the content of the left and right frame name variables

        // first of all we check that one joypad is on the left and the other one is on the right,
        // The openxr frame is oriented as follows X on the right Y upward and Z backward
        // for this reason we take the X coordinate
        //   L       R
        //   |       |       <----- representation of the user from the top
        //   |---H---|              H = head L = left R = right
        //
        //      -|-----> x
        //       |
        //       |
        //       v   z
        const double expectedLeftXCoordinate = getPosition(headOpenXR_T_expectedLeftHandOpenXR)(0);
        const double expectedRightXCoordinate = getPosition(headOpenXR_T_expectedRightHandOpenXR)(0);

        if (expectedLeftXCoordinate * expectedRightXCoordinate > 0)
        {
            yError() << "[JoypadFingersModule::setLeftAndRightJoypad] One joypad should be on the left "
                        "and the other one on the right";
            return false;
        }

        // if the expectedLeftHandOpenXR x coordinate is positive, the left and the right joypad has
        // been swapped. It is not a big deal the following flag is here for handling this case
        // solve your headache
        this->leftAndRightSwapped = expectedLeftXCoordinate > 0;
        if (this->leftAndRightSwapped)
        {
            std::swap(this->leftHandFrameName, this->rightHandFrameName);
        }

        return true;

    }

    bool configureOpenXR(const yarp::os::Searchable& config, const std::string& name)
    {
        if (!this->configureTranformClient(config, name))
        {
            yError() << "[JoypadFingersModule::configureOpenXR] Unable to configure the transform client.";
            return false;
        }

        // Once the vive is stared the left and right joypad are chosen. Since the joypad are
        // exactly the same. We should consider in the application which is the left and the right
        // joypad
        if (!this->setLeftAndRightSwappedFlag())
        {
            yError() << "[JoypadFingersModule::configureOpenXR] Unable to set the flag related "
                        "to the swap of the left and right joypad.";
            return false;
        }

        if (!this->configureJoypad(config, name))
        {
            yError() << "[JoypadFingersModule::configureOpenXR] Unable to configure the joypad client.";
            return false;
        }

        return true;
    }

    double deadzone(const double& input)
    {
        if (input >= 0)
        {
            if (input > this->joypadParameters.deadzone)
                return (input - this->joypadParameters.deadzone)
                       / (this->joypadParameters.fullscale - this->joypadParameters.deadzone);
            else
                return 0.0;
        } else
        {
            if (input < -this->joypadParameters.deadzone)
                return (input + this->joypadParameters.deadzone)
                       / (this->joypadParameters.fullscale - this->joypadParameters.deadzone);
            else
                return 0.0;
        }
    }

    double evaluateDesiredFingersVelocity(const std::vector<int>& fingersSqueezeButtonsMask,
                                          const std::vector<int>& fingersReleaseButtonsMask,
                                          int fingersVelocityIndex)
    {
        double fingersVelocity;
        this->joypadControllerInterface->getAxis(fingersVelocityIndex, fingersVelocity);

        if (fingersVelocity == 0)
            return 0;

        if (this->isButtonStateEqualToMask(fingersSqueezeButtonsMask))
        {
            return fingersVelocity;
        }

        if (this->isButtonStateEqualToMask(fingersReleaseButtonsMask))
        {
            return -fingersVelocity;
        }
        return 0;
    }

    void getDeviceButtonsState()
    {
        if (buttonCount == 0)
        {
            this->joypadControllerInterface->getButtonCount(buttonCount);
            buttonsState.resize(buttonCount);
        }

        float value;
        for (unsigned int i = 0; i < buttonCount; i++)
        {
            this->joypadControllerInterface->getButton(i, value);
            buttonsState[i] = value > 0 ? 1.0 : 0.0;
        }
    }
};

OpenXRJoypadModule::OpenXRJoypadModule()
    : m_pImpl{new Impl()} {};

OpenXRJoypadModule::~OpenXRJoypadModule(){};



bool OpenXRJoypadModule::configure(yarp::os::ResourceFinder& rf)
{
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[JoypadFingersModule::configure] Empty configuration "
                    "for the JoypadFingersModule application.";
        return false;
    }

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_pImpl->dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asFloat64();
    yInfo() << "[JoypadFingersModule::configure] sampling time: " << m_pImpl->dT;

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[JoypadFingersModule::configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    // configure the openxr device
    const std::string headsetGroup = "OPENXR";
    const yarp::os::Bottle& openXROptions = rf.findGroup(headsetGroup);
    if (!m_pImpl->configureOpenXR(openXROptions, getName()))
    {
        yError() << "[JoypadFingersModule::configure] Unable to configure the joypad client";
        return false;
    }

    // configure fingers retargeting
    m_pImpl->leftHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
    leftFingersOptions.append(generalOptions);
    if (!m_pImpl->leftHandFingers->configure(leftFingersOptions, getName()))
    {
        yError() << "[JoypadFingersModule::configure] Unable to initialize the left fingers retargeting.";
    }

    m_pImpl->rightHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
    rightFingersOptions.append(generalOptions);
    if (!m_pImpl->rightHandFingers->configure(rightFingersOptions, getName()))
    {
        yError() << "[JoypadFingersModule::configure] Unable to initialize the right fingers retargeting.";
    }

    // open ports
    std::string portName;

    if (!YarpHelper::getStringFromSearchable(rf, "rpcWalkingPort_name", portName))
    {
        yError() << "[JoypadFingersModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_pImpl->rpcWalkingClient.open("/" + getName() + portName))
    {
        yError() << "[JoypadFingersModule::configure] " << portName << " port already open.";
        return false;
    }

    if(!YarpHelper::getStringFromSearchable(rf, "robotGoalOutputPort_name", portName))
    {
        yError() << "[JoypadFingersModule::configure] Unable to get a string from searchable";
        return false;
    }
    m_pImpl->robotGoalPort.open("/" + getName() + portName);


    m_pImpl->state = Impl::OpenXRFSM::Configured;

    yInfo() << "Configuring done!";

    return true;
}

double OpenXRJoypadModule::getPeriod()
{
    return m_pImpl->dT;
}

bool OpenXRJoypadModule::close()
{
    m_pImpl->joypadDevice.close();
    m_pImpl->transformClientDevice.close();
    m_pImpl->robotGoalPort.close();
    m_pImpl->rpcWalkingClient.close();

    return true;
}


bool OpenXRJoypadModule::updateModule()
{
    // if (m_pImpl->state == Impl::OpenXRFSM::Running)
    // {

        m_pImpl->getDeviceButtonsState();
    //unsigned int axisCount = 0;
        //m_pImpl->joypadControllerInterface->getAxisCount(axisCount);
    //yInfo() << "axis";
    //    for (size_t i = 0; i < axisCount; ++i)
    //    {
    //        double value;
    //        m_pImpl->joypadControllerInterface->getAxis(i, value);
    //        yInfo() << i << "++  " << value;
    //    }

        //yInfo() << "Buttons";
        //size_t i = 0;
        //for (auto button : m_pImpl->buttonsState)
        //    {
        //        yInfo() << i << "++ " << button;
        //        i++;
        //    }

        // send commands to the walking
        double x{0.0}, y{0.0}, z{0.0};

        if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.joypadLeftButtonsMap))
        {
            m_pImpl->joypadControllerInterface->getAxis(m_pImpl->joypadParameters.xJoypadIndex, x);
            m_pImpl->joypadControllerInterface->getAxis(m_pImpl->joypadParameters.yJoypadIndex, y);

            x = m_pImpl->deadzone(x);
            y = -m_pImpl->deadzone(y);
        }

        if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.joypadRightButtonsMap))
        {
            m_pImpl->joypadControllerInterface->getAxis(m_pImpl->joypadParameters.zJoypadIndex, z);
            z = -m_pImpl->deadzone(z);
        }


        // send commands to the walking
        yarp::sig::Vector& goal= m_pImpl->robotGoalPort.prepare();
        goal.clear();
        goal.push_back(x);
        goal.push_back(y);
        goal.push_back(z);
        m_pImpl->robotGoalPort.write();

        // left fingers
        const double leftFingersVelocity = m_pImpl->evaluateDesiredFingersVelocity(
            m_pImpl->joypadParameters.leftFingersSqueezeButtonsMap,
            m_pImpl->joypadParameters.leftFingersReleaseButtonsMap,
            m_pImpl->joypadParameters.fingersVelocityLeftIndex);

        if (!m_pImpl->leftHandFingers->setFingersVelocity(leftFingersVelocity))
        {
            //yError() << "[JoypadFingersModule::updateModule] Unable to set the left finger velocity.";
        }

        if (!m_pImpl->leftHandFingers->move())
        {
            //yError() << "[JoypadFingersModule::updateModule] Unable to move the left finger";
        }

        const double rightFingersVelocity = m_pImpl->evaluateDesiredFingersVelocity(
            m_pImpl->joypadParameters.rightFingersSqueezeButtonsMap,
            m_pImpl->joypadParameters.rightFingersReleaseButtonsMap,
            m_pImpl->joypadParameters.fingersVelocityRightIndex);

        if (!m_pImpl->rightHandFingers->setFingersVelocity(rightFingersVelocity))
        {
            //yError() << "[JoypadFingersModule::updateModule] Unable to set the right finger velocity.";
        }

        if (!m_pImpl->rightHandFingers->move())
        {
            //yError() << "[JoypadFingersModule::updateModule] Unable to move the right finger";
        }

        // check if it is time to stop walking
        if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.stopWalkingButtonsMap))
        {
            yarp::os::Bottle cmd, outcome;
            cmd.addString("stopWalking");
            m_pImpl->rpcWalkingClient.write(cmd, outcome);
            yInfo() << "[JoypadFingersModule::updateModule] stop";
            return false;
        }

    // } else if (m_pImpl->state == Impl::OpenXRFSM::Configured)
    // {
    //     // // check if it is time to prepare or start walking
    //     if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.prepareWalkingButtonsMap))
    //     {
    //         // TODO add a visual feedback for the user
    //         yarp::os::Bottle cmd, outcome;
    //         cmd.addString("prepareRobot");
    //         m_pImpl->rpcWalkingClient.write(cmd, outcome);
    //         m_pImpl->state = OpenXRJoypadModule::Impl::OpenXRFSM::InPreparation;
    //         yInfo() << "[JoypadFingersModule::updateModule] prepare the robot";
    //     }
    // } else if (m_pImpl->state == Impl::OpenXRFSM::InPreparation)
    // {

    //     if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.startWalkingButtonsMap))
    //     {
    //         yarp::os::Bottle cmd, outcome;
    //         cmd.addString("startWalking");
    //         m_pImpl->rpcWalkingClient.write(cmd, outcome);

    //         m_pImpl->state = OpenXRJoypadModule::Impl::OpenXRFSM::Running;
    //         yInfo() << "[JoypadFingersModule::updateModule] start the robot";
    //         yInfo() << "[JoypadFingersModule::updateModule] Running ...";
    //     }
    // }
    return true;
}
