// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <functional>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

// YARP
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>

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
    yarp::sig::Vector m_desiredAcceleration; /**< Desired acceleration for velocity control. */

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
    bool setJointReferences(const yarp::sig::Vector& desiredValue);

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

bool RobotControlHelper::configure(const yarp::os::Searchable& config, const std::string& name)
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
    m_positionFeedbackInDegrees.zero();
    m_positionFeedbackInRadians.resize(m_actuatedDOFs);
    m_positionFeedbackInRadians.zero();
    m_desiredAcceleration.resize(m_actuatedDOFs);
    m_desiredAcceleration = std::numeric_limits<double>::max();

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
        yErrorOnce() << "[RobotControlHelper::switchToControlMode] ControlMode I/F not ready.";
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
        yErrorOnce()
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
    {
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredPosition(i));
    }

    // set desired position
    m_positionDirectInterface->setPositions(m_desiredJointValue.data());

    return true;
}

bool RobotControlHelper::setVelocityReferences(const yarp::sig::Vector& desiredVelocity)
{
    if (m_velocityInterface == nullptr)
    {
        yErrorOnce() << "[RobotControlHelper::setVelocityReferences] Velocity I/F not ready.";
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
    {
        m_desiredJointValue(i) = iDynTree::rad2deg(desiredVelocity(i));
    }

    // since the velocity interface use a minimum jerk trajectory a very high acceleration is set in
    // order to use it as velocity "direct" interface
    m_velocityInterface->setRefAccelerations(m_desiredAcceleration.data());
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

        } else
        {
            limits(i, 0) = iDynTree::deg2rad(minLimitInDegree);
            limits(i, 1) = iDynTree::deg2rad(maxLimitInDegree);
        }
    }
    return true;
}

bool RobotControlHelper::setJointReferences(const yarp::sig::Vector& desiredValue)
{
    switch (m_controlMode)
    {
    case VOCAB_CM_POSITION_DIRECT:
        if (!setDirectPositionReferences(desiredValue))
        {
            yErrorOnce()
                << "[RobotControlHelper::setJointReference] Unable to set the desired joint "
                   "positions";
            return false;
        }
        break;

    case VOCAB_CM_VELOCITY:
        if (!setVelocityReferences(desiredValue))
        {
            yErrorOnce()
                << "[RobotControlHelper::setJointReference] Unable to set the desired joint "
                   "velocities";
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
    bool m_enabled{true}; /**< Flag to enable/disable the control of the fingers. */

public:
    /**
     * Configure the object.
     * @param config reference to a resource finder object.
     * @param name name of the robot
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name);

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
    bool move();
};

bool FingersRetargeting::configure(const yarp::os::Searchable& config, const std::string& name)
{
    m_enabled = config.check("enabled", yarp::os::Value(true)).asBool();

    if (!m_enabled)
    {
        return true;
    }

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
    if (!m_enabled)
    {
        return true;
    }

    if (m_desiredJointValue.size() == 0)
    {
        yErrorOnce() << "The FingerRetargeting object has not been properly initialized.";
        return false;
    }

    if (m_controlHelper->isVelocityControlUsed())
    {
        m_desiredJointValue = fingersVelocity * m_fingersScaling;
    } else
    {
        if (m_fingerIntegrator == nullptr)
        {
            yError()
                << "[FingersRetargeting::setFingersVelocity] The integrator is not initialized."
                   "Please call configure() method";
            return false;
        }
        m_desiredJointValue = m_fingerIntegrator->integrate(fingersVelocity * m_fingersScaling);
    }
    return true;
}

bool FingersRetargeting::move()
{
    if (!m_enabled)
    {
        return true;
    }

    return m_controlHelper->setJointReferences(m_desiredJointValue);
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
        size_t startWalkingActiveButtons;
        size_t prepareWalkingActiveButtons;
        size_t stopWalkingActiveButtons;
        std::vector<int> walkingCommandReleaseButtonMap;
        std::vector<int> leftFingersSqueezeButtonsMap;
        std::vector<int> leftFingersReleaseButtonsMap;
        std::vector<int> rightFingersSqueezeButtonsMap;
        std::vector<int> rightFingersReleaseButtonsMap;

        enum class InputMode
        {
            Axis,
            Stick
        };

        struct InputAxis
        {
            int index;
            int dof{0};
            InputMode mode;
            double sign{+1.0};
            yarp::sig::Vector buffer;
        };

        InputAxis xInput;
        InputAxis yInput;
        InputAxis zInput;

        std::vector<int> joypadLeftButtonsMap;
        std::vector<int> joypadRightButtonsMap;

        InputAxis
            leftFingersVelocityInput; /**< Index of the trigger used for squeezing the left hand */
        InputAxis rightFingersVelocityInput; /**< Index of the trigger used for
                                                  squeezing the right hand */
    };
    JoypadParameters joypadParameters;

    double dT;
    bool print_buttons{false};
    bool print_axes{false};
    bool print_sticks{false};

    std::unique_ptr<FingersRetargeting> leftHandFingers; /**< Pointer to the left
                                                              finger retargeting object. */
    std::unique_ptr<FingersRetargeting> rightHandFingers; /**< Pointer to the right
                                                               finger retargeting object. */

    yarp::os::RpcClient rpcWalkingClient; /**< Rpc client used for sending command to the walking
                                               controller */
    yarp::os::BufferedPort<yarp::sig::Vector>
        robotGoalPort; /**< Port used to specify the desired goal position. */

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
    unsigned int buttonCount{0};
    unsigned int axisCount{0};
    unsigned int sticksCount{0};
    std::string walkingCommand;
    size_t activeButtons{0};

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
        yarp::os::Property options;
        options.put("device", config.check("transform_server_device", yarp::os::Value("frameTransformClient")).asString());
        options.put("filexml_option",  config.check("transform_server_file", yarp::os::Value("ftc_yarp_only.xml")).asString());
        options.put("ft_client_prefix", config.check("transform_server_local", yarp::os::Value(applicationName + "/tf")).asString());
        if (config.check("transform_server_remote"))
        {
            options.put("ft_server_prefix", config.find("transform_server_remote").asString());
        }
        options.put("local_rpc", "/" + applicationName + "/tf/local_rpc");

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
            yError()
                << "[JoypadFingersModule::configureTranformClient] Cannot obtain Transform client.";
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
            yError()
                << "[JoypadFingersModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, //
                                                 "right_hand_frame_name",
                                                 this->rightHandFrameName))
        {
            yError()
                << "[JoypadFingersModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        return true;
    }

    bool getIndexCode(const yarp::os::Searchable& config,
                      const std::string& tag,
                      JoypadParameters::InputAxis& output)
    {
        std::string code;
        if (!YarpHelper::getStringFromSearchable(config, //
                                                 tag,
                                                 code))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter" << tag;
            return false;
        }

        if (code.size() < 2)
        {
            yError() << "[JoypadFingersModule::configureJoypad] The code of" << tag
                     << "is too short. "
                        "The first two characters are supposed "
                        "to be either \"A\" or \"S\", and the second character"
                        "either \"+\" or \"-\". Input:"
                     << code;
            return false;
        }

        char type = code[0];
        char sign = code[1];

        if (sign == '+')
        {
            output.sign = +1.0;
        } else if (sign == '-')
        {
            output.sign = -1.0;
        } else
        {
            yError() << "[JoypadFingersModule::configureJoypad] The second character of" << tag
                     << "is supposed to be"
                        "either \"+\" or \"-\". Input:"
                     << code;
            return false;
        }

        if (type == 'A')
        {
            int axis = std::stoi(code.substr(2));

            if (axis >= axisCount)
            {
                yError() << "[JoypadFingersModule::configureJoypad] The selected axis for" << tag
                         << "is out of bounds. Axis:" << axis << "Available:" << axisCount;
                return false;
            }

            output.index = axis;
            output.mode = JoypadParameters::InputMode::Axis;
            yInfo() << "Selected axis index" << axis << "for" << tag << "(sign" << sign << ").";
        } else if (type == 'S')
        {
            size_t dotIndex = code.find('.');
            if (dotIndex == std::string::npos)
            {
                yError() << "The code for" << tag
                         << "selects a stick, but no dot is found. Input:" << code;
                return false;
            }

            int stick = std::stoi(code.substr(2, dotIndex - 2));
            int dof = std::stoi(code.substr(dotIndex + 1));

            if (stick >= sticksCount)
            {
                yError() << "[JoypadFingersModule::configureJoypad] The selected stick for" << tag
                         << "is out of bounds. Stick:" << stick << "Available:" << sticksCount;
                return false;
            }

            unsigned int maxDofs{0};

            if (!joypadControllerInterface->getStickDoF(stick, maxDofs))
            {
                yError() << "[JoypadFingersModule::configureJoypad] Could not get the number of "
                            "DOFs for stick:"
                         << stick;
                return false;
            }

            if (dof >= maxDofs)
            {
                yError() << "[JoypadFingersModule::configureJoypad] The selected dof stick for"
                         << tag << "is out of bounds. Stick:" << stick << "dof:" << dof
                         << "Available:" << maxDofs;
                return false;
            }

            output.index = stick;
            output.dof = dof;
            output.mode = JoypadParameters::InputMode::Stick;
            output.buffer.reserve(maxDofs);
            output.buffer.zero();
            yInfo() << "Selected stick index" << stick << "(dof" << dof << ") for" << tag << "(sign"
                    << sign << ").";

        } else
        {
            yError() << "[JoypadFingersModule::configureJoypad] The first character of" << tag
                     << "is supposed "
                        "to be either \"A\" or \"S\". Input"
                     << code;
            return false;
        }

        return true;
    };

    bool configureJoypad(const yarp::os::Searchable& config, const std::string& name)
    {
        std::string joypadDeviceRemote;
        if (!YarpHelper::getStringFromSearchable(config,
                                                 "joypad_device_remote", //
                                                 joypadDeviceRemote))
        {
            yError() << "[JoypadFingersModule::configureJoypad] Unable to find parameter "
                        "joypad_device_remote";
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

        this->joypadControllerInterface->getButtonCount(buttonCount);
        buttonsState.resize(buttonCount);
        this->joypadControllerInterface->getAxisCount(axisCount);
        this->joypadControllerInterface->getStickCount(sticksCount);

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

        JoypadParameters::InputAxis rightXCode;
        std::vector<std::pair<std::string, JoypadParameters::InputAxis&>> codes
            = {{"left_x_code", this->joypadParameters.xInput},
               {"left_y_code", this->joypadParameters.yInput},
               {"right_x_code", rightXCode},
               {"right_y_code", this->joypadParameters.zInput},
               {"left_fingers_velocity_code", this->joypadParameters.leftFingersVelocityInput},
               {"right_fingers_velocity_code", this->joypadParameters.rightFingersVelocityInput}};

        for (auto& c : codes)
        {
            if (!getIndexCode(config, c.first, c.second))
            {
                yError() << "[JoypadFingersModule::configureJoypad] Unable to parse parameter"
                         << c.first;
                return false;
            }
        }

        std::vector<int> prepareWalkingButtonsSwappedMap;
        std::vector<int> stopWalkingButtonsSwappedMap;
        std::vector<int> startWalkingButtonsSwappedMap;

        std::vector<std::pair<std::string, std::vector<int>&>> buttonsMaps
            = {{"start_walking_buttons_map", this->joypadParameters.startWalkingButtonsMap},
               {"stop_walking_buttons_map", this->joypadParameters.stopWalkingButtonsMap},
               {"prepare_walking_buttons_map", this->joypadParameters.prepareWalkingButtonsMap},
               {"start_walking_buttons_map_swapped", startWalkingButtonsSwappedMap},
               {"stop_walking_buttons_swapped_map", stopWalkingButtonsSwappedMap},
               {"prepare_walking_buttons_swapped_map", prepareWalkingButtonsSwappedMap},
               {"walking_command_release_buttons_map",
                this->joypadParameters.walkingCommandReleaseButtonMap},
               {"left_fingers_squeeze_buttons_map",
                this->joypadParameters.leftFingersSqueezeButtonsMap},
               {"left_fingers_release_buttons_map",
                this->joypadParameters.leftFingersReleaseButtonsMap},
               {"right_fingers_squeeze_buttons_map",
                this->joypadParameters.rightFingersSqueezeButtonsMap},
               {"right_fingers_release_buttons_map",
                this->joypadParameters.rightFingersReleaseButtonsMap},
               {"left_walking_buttons_map", this->joypadParameters.joypadLeftButtonsMap},
               {"right_walking_buttons_map", this->joypadParameters.joypadRightButtonsMap}};

        for (auto& b : buttonsMaps)
        {
            if (!YarpHelper::getIntVectorFromSearchable(config, b.first, b.second))
            {
                yError() << "[JoypadFingersModule::configureJoypad] Unable to parse parameter"
                         << b.first;
                return false;
            }
        }

        // if swapped we have to swap all the maps
        if (this->leftAndRightSwapped)
        {
            std::swap(this->joypadParameters.rightFingersReleaseButtonsMap,
                      this->joypadParameters.leftFingersReleaseButtonsMap);

            std::swap(this->joypadParameters.rightFingersSqueezeButtonsMap,
                      this->joypadParameters.leftFingersSqueezeButtonsMap);

            std::swap(this->joypadParameters.rightFingersVelocityInput,
                      this->joypadParameters.leftFingersVelocityInput);

            std::swap(this->joypadParameters.joypadRightButtonsMap,
                      this->joypadParameters.joypadLeftButtonsMap);

            std::swap(this->joypadParameters.yInput, this->joypadParameters.zInput);

            this->joypadParameters.xInput = rightXCode;

            this->joypadParameters.startWalkingButtonsMap = startWalkingButtonsSwappedMap;
            this->joypadParameters.stopWalkingButtonsMap = stopWalkingButtonsSwappedMap;
            this->joypadParameters.prepareWalkingButtonsMap = prepareWalkingButtonsSwappedMap;
        }

        auto countActiveButtons = [](const std::vector<int>& input) -> size_t {
            size_t output = 0;
            for (auto b : input)
            {
                if (b > 0)
                {
                    output++;
                }
            }
            return output;
        };

        this->joypadParameters.startWalkingActiveButtons
            = countActiveButtons(this->joypadParameters.startWalkingButtonsMap);
        this->joypadParameters.stopWalkingActiveButtons
            = countActiveButtons(this->joypadParameters.stopWalkingButtonsMap);
        this->joypadParameters.prepareWalkingActiveButtons
            = countActiveButtons(this->joypadParameters.prepareWalkingButtonsMap);

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

            ok = ok
                 && this->frameTransformInterface->getTransform(this->rightHandFrameName, //
                                                                this->headFrameName,
                                                                headOpenXR_T_rightHandOpenXR);

            ok = ok
                 && this->frameTransformInterface->getTransform(this->leftHandFrameName, //
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
        while (!readTransforms(headOpenXR_T_expectedLeftHandOpenXR,
                               headOpenXR_T_expectedRightHandOpenXR))
        {
            if (++counter == maxAttempt)
            {
                yError() << "[JoypadFingersModule::setLeftAndRightSwappedFlag] Unable to read the "
                            "transform client.";
                return false;
            }

            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        auto getPosition = [](const yarp::sig::Matrix& m) {
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
        const double expectedRightXCoordinate
            = getPosition(headOpenXR_T_expectedRightHandOpenXR)(0);

        if (expectedLeftXCoordinate * expectedRightXCoordinate > 0)
        {
            yError() << "[JoypadFingersModule::setLeftAndRightSwappedFlag] One joypad should be on "
                        "the left "
                        "and the other one on the right";
            return false;
        }

        // if the expectedLeftHandOpenXR x coordinate is positive, the left and the right joypad has
        // been swapped. It is not a big deal the following flag is here for handling this case
        // solve your headache
        this->leftAndRightSwapped = expectedLeftXCoordinate > 0;

        if (this->leftAndRightSwapped)
        {
            yInfo() << "[JoypadFingersModule::setLeftAndRightSwappedFlag] The hands are swapped!";
        }

        return true;
    }

    bool configureOpenXR(const yarp::os::Searchable& config, const std::string& name)
    {
        if (config.check("check_hands_inverted", yarp::os::Value(false)).asBool())
        {
            if (!this->configureTranformClient(config, name))
            {
                yError() << "[JoypadFingersModule::configureOpenXR] Unable to configure the "
                            "transform client.";
                return false;
            }

            // Once the vive is stared the left and right joypad are chosen. Since the joypad are
            // exactly the same. We should consider in the application which is the left and the
            // right joypad
            if (!this->setLeftAndRightSwappedFlag())
            {
                yError() << "[JoypadFingersModule::configureOpenXR] Unable to set the flag related "
                            "to the swap of the left and right joypad.";
                return false;
            }
        }

        if (!this->configureJoypad(config, name))
        {
            yError()
                << "[JoypadFingersModule::configureOpenXR] Unable to configure the joypad client.";
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

    double getInputValue(JoypadParameters::InputAxis& input)
    {
        double value{0.0};
        switch (input.mode)
        {
        case JoypadParameters::InputMode::Axis:
            joypadControllerInterface->getAxis(input.index, value);
            value = input.sign * deadzone(value);
            return value;
        case JoypadParameters::InputMode::Stick:
            joypadControllerInterface->getStick(
                input.index, input.buffer, yarp::dev::IJoypadController::JypCtrlcoord_CARTESIAN);
            value = input.sign * deadzone(input.buffer[input.dof]);
            return value;
        default:
            return value;
        }
    }

    double evaluateDesiredFingersVelocity(const std::vector<int>& fingersSqueezeButtonsMask,
                                          const std::vector<int>& fingersReleaseButtonsMask,
                                          JoypadParameters::InputAxis& fingersVelocityCode)
    {
        double fingersVelocity = getInputValue(fingersVelocityCode);

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

    m_pImpl->print_buttons = rf.check("print_buttons", yarp::os::Value(false)).asBool();
    m_pImpl->print_axes = rf.check("print_axes", yarp::os::Value(false)).asBool();
    m_pImpl->print_sticks = rf.check("print_sticks", yarp::os::Value(false)).asBool();

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
        yError() << "[JoypadFingersModule::configure] Unable to initialize the left fingers "
                    "retargeting.";
    }

    m_pImpl->rightHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
    rightFingersOptions.append(generalOptions);
    if (!m_pImpl->rightHandFingers->configure(rightFingersOptions, getName()))
    {
        yError() << "[JoypadFingersModule::configure] Unable to initialize the right fingers "
                    "retargeting.";
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

    if (!YarpHelper::getStringFromSearchable(rf, "robotGoalOutputPort_name", portName))
    {
        yError() << "[JoypadFingersModule::configure] Unable to get a string from searchable";
        return false;
    }
    m_pImpl->robotGoalPort.open("/" + getName() + portName);

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
    m_pImpl->getDeviceButtonsState();

    if (m_pImpl->print_axes)
    {
        for (size_t i = 0; i < m_pImpl->axisCount; ++i)
        {
            double value;
            m_pImpl->joypadControllerInterface->getAxis(i, value);
            yInfo() << "Axis [" << i << "]:" << value;
        }
    }

    if (m_pImpl->print_buttons)
    {
        for (size_t i = 0; i < m_pImpl->buttonsState.size(); ++i)
        {
            yInfo() << "Button [" << i << "]:" << m_pImpl->buttonsState[i];
        }
    }

    if (m_pImpl->print_sticks)
    {
        for (size_t i = 0; i < m_pImpl->sticksCount; ++i)
        {
            yarp::sig::Vector dofs;
            m_pImpl->joypadControllerInterface->getStick(
                i, dofs, yarp::dev::IJoypadController::JypCtrlcoord_CARTESIAN);
            std::stringstream vector_string;
            for (size_t j = 0; j < dofs.size(); ++j)
            {
                vector_string << "(dof " << j << ") " << dofs[j] << " ";
            }
            yInfo() << "Stick [" << i << "]:" << vector_string.str();
        }
    }

    // send commands to the walking
    double x{0.0}, y{0.0}, z{0.0};

    if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.joypadLeftButtonsMap))
    {
        x = m_pImpl->getInputValue(m_pImpl->joypadParameters.xInput);
        y = m_pImpl->getInputValue(m_pImpl->joypadParameters.yInput);
    }

    if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.joypadRightButtonsMap))
    {
        z = m_pImpl->getInputValue(m_pImpl->joypadParameters.zInput);
    }

    // send commands to the walking
    yarp::sig::Vector& goal = m_pImpl->robotGoalPort.prepare();
    goal.clear();
    goal.push_back(x);
    goal.push_back(y);
    goal.push_back(z);
    m_pImpl->robotGoalPort.write();

    // left fingers
    const double leftFingersVelocity = m_pImpl->evaluateDesiredFingersVelocity(
        m_pImpl->joypadParameters.leftFingersSqueezeButtonsMap,
        m_pImpl->joypadParameters.leftFingersReleaseButtonsMap,
        m_pImpl->joypadParameters.leftFingersVelocityInput);

    m_pImpl->leftHandFingers->setFingersVelocity(leftFingersVelocity);
    m_pImpl->leftHandFingers->move();

    const double rightFingersVelocity = m_pImpl->evaluateDesiredFingersVelocity(
        m_pImpl->joypadParameters.rightFingersSqueezeButtonsMap,
        m_pImpl->joypadParameters.rightFingersReleaseButtonsMap,
        m_pImpl->joypadParameters.rightFingersVelocityInput);

    m_pImpl->rightHandFingers->setFingersVelocity(rightFingersVelocity);
    m_pImpl->rightHandFingers->move();

    if (!m_pImpl->walkingCommand.empty()
        && m_pImpl->isButtonStateEqualToMask(
            m_pImpl->joypadParameters.walkingCommandReleaseButtonMap))
    {
        yarp::os::Bottle cmd, outcome;
        cmd.addString(m_pImpl->walkingCommand);
        m_pImpl->rpcWalkingClient.write(cmd, outcome);
        yInfo() << "[OpenXRJoypadModule::updateModule] Sent" << m_pImpl->walkingCommand;
        m_pImpl->walkingCommand = "";
        m_pImpl->activeButtons = 0;
    }

    // The activeButtons logic is to make sure that the command with more buttons is privileged.
    // This is to make easy to trigger commands that require combinations of buttons
    // that would trigger other commands if not released all at the same time

    if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.stopWalkingButtonsMap)
        && (m_pImpl->joypadParameters.stopWalkingActiveButtons > m_pImpl->activeButtons))
    {
        m_pImpl->activeButtons = m_pImpl->joypadParameters.stopWalkingActiveButtons;
        m_pImpl->walkingCommand = "stopWalking";
    }

    if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.prepareWalkingButtonsMap)
        && (m_pImpl->joypadParameters.prepareWalkingActiveButtons > m_pImpl->activeButtons))
    {
        m_pImpl->activeButtons = m_pImpl->joypadParameters.prepareWalkingActiveButtons;
        m_pImpl->walkingCommand = "prepareRobot";
    }

    if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.startWalkingButtonsMap)
        && (m_pImpl->joypadParameters.startWalkingActiveButtons > m_pImpl->activeButtons))
    {
        m_pImpl->activeButtons = m_pImpl->joypadParameters.startWalkingActiveButtons;
        m_pImpl->walkingCommand = "startWalking";
    }

    return true;
}
