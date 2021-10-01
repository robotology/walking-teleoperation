/**
 * @file OpenXRModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
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

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <OpenXRModule.hpp>
#include <Utils.hpp>

Eigen::Ref<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> getRotation(yarp::sig::Matrix& m)
{
    return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data()).topLeftCorner<3, 3>();
}

auto getPosition(yarp::sig::Matrix& m)
{
    return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data())
        .topRightCorner<3, 1>();
}

auto getPosition(const yarp::sig::Matrix& m)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(m.data())
        .topRightCorner<3, 1>();
}

yarp::sig::Matrix identitySE3()
{
    yarp::sig::Matrix m;
    m.resize(4, 4);
    getRotation(m).setIdentity();
    getPosition(m).setZero();
    m(3, 3) = 1;
    return m;
}

struct OpenXRModule::Impl
{
    struct JoypadParameters
    {
        double deadzone; /**< Joypad deadzone */
        double fullscale; /**< Joypad fullscale */
        double scaleX; /**< Scaling factor on the x axis */
        double scaleY; /**< Scaling factor on the y axis */
        double x; /**< x value */
        double y; /**< y value */

        std::vector<int> startWalkingButtonsMap;
        std::vector<int> prepareWalkingButtonsMap;
        std::vector<int> stopWalkingButtonsMap;
        std::vector<int> leftFingersSqueezeButtonsMap;
        std::vector<int> leftFingersReleaseButtonsMap;
        std::vector<int> rightFingersSqueezeButtonsMap;
        std::vector<int> rightFingersReleaseButtonsMap;

        int xJoypadIndex; /**< Mapping of the axis related to x coordinate */
        int yJoypadIndex; /**< Mapping of the axis related to y coordinate */
        std::vector<int> joypadButtonsMap;

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

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> neckJointsPreparationSmoother{nullptr};
    double preparationSmoothingTime;
    double dT;
    unsigned actuatedDOFs;
    double playerOrientationThreshold;

    std::unique_ptr<HeadRetargeting> head; /**< Pointer to the head retargeting object. */
    std::unique_ptr<FingersRetargeting> leftHandFingers; /**< Pointer to the left
                                                              finger retargeting object. */
    std::unique_ptr<FingersRetargeting> rightHandFingers; /**< Pointer to the right
                                                               finger retargeting object. */
    std::unique_ptr<HandRetargeting> rightHand; /**< Pointer to the right
                                                     hand retargeting object. */
    std::unique_ptr<HandRetargeting> leftHand; /**< Pointer to the left hand
                                                    retargeting object. */

    yarp::os::BufferedPort<yarp::sig::Vector> leftHandPosePort; /**< Left hand port pose. */
    yarp::os::BufferedPort<yarp::sig::Vector> rightHandPosePort; /**< Right hand port pose. */

    yarp::os::RpcClient rpcWalkingClient; /**< Rpc client used for sending command to the walking
                                               controller */

    // transform server
    yarp::dev::PolyDriver transformClientDevice; /**< Transform client. */
    yarp::dev::IFrameTransform* frameTransformInterface{nullptr}; /**< Frame transform
                                                                       interface. */
    yarp::dev::PolyDriver joypadDevice; /**< Joypad polydriver. */
    yarp::dev::IJoypadController* joypadControllerInterface{nullptr}; /**< joypad interface. */

    std::string rootFrameName; /**< Name of the root frame used in the transform server */
    std::string headFrameName; /**< Name of the head frame used in the transform server */
    std::string leftHandFrameName; /**< Name of the left hand frame used in the transform server */
    std::string rightHandFrameName; /**< Name of the right hand
                                       frame used in the transform server */

    yarp::sig::Matrix openXRRoot_T_lOpenXR;
    yarp::sig::Matrix openXRRoot_T_rOpenXR;
    yarp::sig::Matrix openXRRoot_T_headOpenXR;
    yarp::sig::Matrix openXRInitialAlignement;

    std::vector<double> openXRHeadsetPoseInertial;

    bool moveRobot{false};
    double playerOrientation{0}; /**< Player orientation (read by the Virtualizer)
                                     only yaw. */

    bool leftAndRightSwapped{false};
    std::vector<int> buttonsState;

    bool isButtonStateEqualToMask(const std::vector<int>& mask) const
    {
        return (mask.size() == this->buttonsState.size()
                && std::equal(mask.begin(),
                              mask.end(),
                              this->buttonsState.begin(),
                              [](const auto& a, const auto& b) {
                                  if (a < 0)
                                      return true;
                                  return a == b;
                              }));
    }

    void initializeNeckJointsSmoother(const unsigned actuatedDOFs,
                                      const double dT,
                                      const double smoothingTime,
                                      const yarp::sig::Vector jointsInitialValue)
    {
        this->neckJointsPreparationSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(
            this->actuatedDOFs, this->dT, smoothingTime);
        this->neckJointsPreparationSmoother->init(jointsInitialValue);
    }

    void getNeckJointsRefSmoothedValues(yarp::sig::Vector& smoothedJointValues)
    {
        yarp::sig::Vector jointValues = {0.0, 0.0, 0.0};
        this->neckJointsPreparationSmoother->computeNextValues(jointValues);
        smoothedJointValues = this->neckJointsPreparationSmoother->getPos();
    }

    bool getFeedbacks()
    {
        if (!this->head->controlHelper()->getFeedback())
        {
            yError() << "[OpenXRModule::getFeedbacks] Unable to get the joint encoders feedback: "
                        "HeadRetargeting";
            return false;
        }
        this->head->controlHelper()->updateTimeStamp();

        return true;
    }

    bool getTransforms()
    {
        // check if everything is ok
        if (!this->frameTransformInterface->frameExists(this->rootFrameName))
        {
            yError() << "[OpenXRModule::getTransforms] No " << this->rootFrameName << " frame.";
            return false;
        }

        if (!this->frameTransformInterface->frameExists(this->headFrameName))
        {
            yError() << "[OpenXRModule::getTransforms] If OpenXr is used the head transform "
                        "must be provided by the transform server.";
            return false;
        }

        if (!this->frameTransformInterface->getTransform(this->headFrameName, //
                                                         this->rootFrameName,
                                                         this->openXRRoot_T_headOpenXR))
        {
            yError() << "[OpenXRModule::getTransforms] Unable to evaluate the "
                     << this->headFrameName << " to " << this->rootFrameName << "transformation";
            return false;
        }

        // This is to remove any initial misplacement and rotations around gravity
        iDynTree::toEigen(this->openXRRoot_T_headOpenXR)
            = iDynTree::toEigen(this->openXRInitialAlignement)
              * iDynTree::toEigen(this->openXRRoot_T_headOpenXR);

        iDynTree::Rotation temp;
        iDynTree::toEigen(temp)
            = iDynTree::toEigen(this->openXRRoot_T_headOpenXR).topLeftCorner<3, 3>();

        temp.getRPY(this->openXRHeadsetPoseInertial[3],
                    this->openXRHeadsetPoseInertial[4],
                    this->openXRHeadsetPoseInertial[5]);
        // this->openXRHeadsetPoseInertial is a 6d std::vector here I'm getting the first three
        // elements
        Eigen::Map<Eigen::Vector3d>(this->openXRHeadsetPoseInertial.data())
            = getPosition(this->openXRRoot_T_headOpenXR);

        if (!this->frameTransformInterface->frameExists(this->leftHandFrameName))
        {

            yError() << "[OpenXRModule::getTransforms] No " << this->leftHandFrameName << "frame.";
            return false;
        }

        if (!this->frameTransformInterface->frameExists(this->rightHandFrameName))
        {
            yError() << "[OpenXRModule::getTransforms] No " << this->rightHandFrameName
                     << " frame.";
            return false;
        }

        if (!this->frameTransformInterface->getTransform(this->leftHandFrameName, //
                                                         this->rootFrameName,
                                                         this->openXRRoot_T_lOpenXR))
        {
            yError() << "[OpenXRModule::getTransforms] Unable to evaluate the "
                     << this->leftHandFrameName << " to " << this->rootFrameName
                     << "transformation";
            return false;
        }

        if (!this->frameTransformInterface->getTransform(this->rightHandFrameName, //
                                                         this->rootFrameName,
                                                         this->openXRRoot_T_rOpenXR))
        {
            yError() << "[OpenXRModule::getTransforms] Unable to evaluate the "
                     << this->rightHandFrameName << " to " << this->rootFrameName
                     << "transformation";
            return false;
        }

        // This is to remove any initial misplacement and rotations around gravity
        iDynTree::toEigen(this->openXRRoot_T_rOpenXR)
            = iDynTree::toEigen(this->openXRInitialAlignement)
              * iDynTree::toEigen(this->openXRRoot_T_rOpenXR);

        iDynTree::toEigen(this->openXRRoot_T_lOpenXR)
            = iDynTree::toEigen(this->openXRInitialAlignement)
              * iDynTree::toEigen(this->openXRRoot_T_lOpenXR);

        return true;
    }

    bool configureTranformClient(const yarp::os::Searchable& config,
                                 const std::string& applicationName)
    {
        yarp::os::Property options;
        options.put("device", "transformClient");
        options.put("remote", "/transformServer");
        options.put("local", "/" + applicationName + "/transformClient");

        if (!this->transformClientDevice.open(options))
        {
            yError() << "[OpenXRModule::configureTranformClient] Unable "
                        "to open transformClient device";
            return false;
        }

        // obtain the interface
        if (!this->transformClientDevice.view(this->frameTransformInterface)
            || this->frameTransformInterface == nullptr)
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, "root_frame_name", this->rootFrameName))
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, "head_frame_name", this->headFrameName))
        {

            yError() << "[OpenXRModule::configureTranformClient] Seems that the head "
                        "orientation is not streamed through the transform server.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, //
                                                 "left_hand_frame_name",
                                                 this->leftHandFrameName))
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(config, //
                                                 "right_hand_frame_name",
                                                 this->rightHandFrameName))
        {
            yError() << "[OpenXRModule::configureTranformClient] Cannot obtain Transform client.";
            return false;
        }

        this->openXRRoot_T_lOpenXR = identitySE3();
        this->openXRRoot_T_rOpenXR = identitySE3();
        this->openXRRoot_T_headOpenXR = identitySE3();
        this->openXRInitialAlignement = identitySE3();

        return true;
    }

    bool configureJoypad(const yarp::os::Searchable& config, const std::string& name)
    {
        yarp::os::Property options;
        options.put("device", "JoypadControlClient");
        options.put("remote", "/joypadDevice/Oculus");
        options.put("local", "/" + name + "/joypadControlClient");

        if (!this->joypadDevice.open(options))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to open the polydriver.";
            return false;
        }

        // get the interface
        if (!this->joypadDevice.view(this->joypadControllerInterface)
            || this->joypadControllerInterface == nullptr)
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to attach JoypadController"
                        " interface to the PolyDriver object";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config,
                                                 "deadzone", //
                                                 this->joypadParameters.deadzone))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter deadzone";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config, //
                                                 "fullscale",
                                                 this->joypadParameters.fullscale))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter fullscale";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config, "scale_X", this->joypadParameters.scaleX))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter scale_X";
            return false;
        }

        if (!YarpHelper::getDoubleFromSearchable(config, "scale_Y", this->joypadParameters.scaleY))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter scale_Y";
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
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
                        "start_walking_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.startWalkingButtonsMap);
        stateMachineButtonsMap.push_back(&this->joypadParameters.startWalkingButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "prepare_walking_buttons_map",
                                                    this->joypadParameters.prepareWalkingButtonsMap))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
                        "prepare_walking_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.prepareWalkingButtonsMap);
        stateMachineButtonsMap.push_back(&this->joypadParameters.prepareWalkingButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "left_fingers_squeeze_buttons_map",
                                                    this->joypadParameters.leftFingersSqueezeButtonsMap))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
                        "left_fingers_squeeze_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.leftFingersSqueezeButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "left_fingers_release_buttons_map",
                                                    this->joypadParameters.leftFingersReleaseButtonsMap))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
                        "left_fingers_release_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.leftFingersReleaseButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "right_fingers_squeeze_buttons_map",
                                                    this->joypadParameters.rightFingersSqueezeButtonsMap))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
                        "right_fingers_squeeze_buttons_map";
            return false;
        }
        buttonsMap.push_back(&this->joypadParameters.rightFingersSqueezeButtonsMap);

        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "right_fingers_release_buttons_map",
                                                    this->joypadParameters.rightFingersReleaseButtonsMap))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
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
                    << "[OpenXRModule::configureJoypad] Mismatch in the size of the buttons map. "
                       "Please check the configuration file";
                return false;
            }
        }

        std::vector<int> leftWalkingButtonsMap;
        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "left_walking_buttons_map",
                                                    leftWalkingButtonsMap))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
                        "left_walking_buttons_map";
            return false;
        }
        buttonsMap.push_back(&leftWalkingButtonsMap);

        std::vector<int> rightWalkingButtonsMap;
        if (!YarpHelper::getIntVectorFromSearchable(config, //
                                                    "right_walking_buttons_map",
                                                    rightWalkingButtonsMap))
        {
            yError() << "[OpenXRModule::configureJoypad] Unable to find parameter "
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
                    yError() << "[OpenXRModule::configureJoypad] State machine maps cannot must be "
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

        this->joypadParameters.fingersVelocityLeftIndex = !this->leftAndRightSwapped ? 0 : 1;
        this->joypadParameters.fingersVelocityRightIndex = !this->leftAndRightSwapped ? 1 : 0;

        constexpr bool useLeftStick = true;
        constexpr int leftXIndex = 2;
        constexpr int leftYIndex = 3;
        constexpr int rightXIndex = 4;
        constexpr int rightYIndex = 5;

        // truth table
        // use left | left_and_right_swapped  | output
        //   T      |       T                 |  use right index (F)
        //   T      |       F                 |  use left index (T)
        //   F      |       T                 |  use left index (T)
        //   F      |       F                 |  use right index (F)
        // this is the xor operator
        // in c++ you can use !A != !B
        this->joypadParameters.xJoypadIndex
            = (!useLeftStick != !this->leftAndRightSwapped) ? leftXIndex : rightXIndex;
        this->joypadParameters.yJoypadIndex
            = (!useLeftStick != !this->leftAndRightSwapped) ? leftYIndex : rightYIndex;
        this->joypadParameters.joypadButtonsMap = (!useLeftStick != !this->leftAndRightSwapped)
                                                      ? leftWalkingButtonsMap
                                                      : rightWalkingButtonsMap;

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

            ok = ok && this->frameTransformInterface->frameExists(this->rootFrameName);
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
        constexpr unsigned int maxAttempt = 20;
        while (!readTransforms(headOpenXR_T_expectedLeftHandOpenXR, headOpenXR_T_expectedRightHandOpenXR))
        {
            if (++counter == maxAttempt)
            {
                yError() << "[OpenXRModule::setLeftAndRightJoypad] Unable to read the "
                            "transform client.";
                return false;
            }

            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }

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
            yError() << "[OpenXRModule::setLeftAndRightJoypad] One joypad should be on the left "
                        "and the other one on the right";
            return false;
        }

        // if the expectedLeftHandOpenXR x coordinate is positive, the left and the right joypad has
        // been swapped. It is not a big deal the following flag is here for handling this case
        // solve your hadake
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
            yError() << "[OpenXRModule::configureOpenXR] Unable to configure the transform client.";
            return false;
        }

        // Once the vive is stared the left and right joypad are chosen. Since the joypad are
        // exactly the same. We should consider in the application which is the left and the right
        // joypad
        if (!this->setLeftAndRightSwappedFlag())
        {
            yError() << "[OpenXRModule::configureOpenXR] Unable to set the flag related "
                        "to the swap of the left and right joypad.";
            return false;
        }

        if (!this->configureJoypad(config, name))
        {
            yError() << "[OpenXRModule::configureOpenXR] Unable to configure the joypad client.";
            return false;
        }

        return true;
    }

    bool resetCamera(const std::string& cameraPort, const std::string& localPort)
    {
        yarp::dev::PolyDriver grabberDriver;

        yarp::os::Property config;
        config.put("device", "remote_grabber");
        config.put("remote", cameraPort);
        config.put("local", localPort);

        bool opened = grabberDriver.open(config);
        if (!opened)
        {
            yError() << "[OpenXRModule::configure] Cannot open remote_grabber device on port "
                     << cameraPort << ".";
            return false;
        }

        yarp::dev::IFrameGrabberControlsDC1394* grabberInterface;

        if (!grabberDriver.view(grabberInterface) || !grabberInterface)
        {
            yError() << "[OpenXRModule::configure] RemoteGrabber does not have "
                        "IFrameGrabberControlDC1394 interface, please update yarp.";
            return false;
        }

        if (!grabberInterface->setResetDC1394())
        {
            yError() << "[OpenXRModule::configure] Failed to reset the camera on port "
                     << cameraPort << ".";
            return false;
        }

        grabberDriver.close();

        return true;
    };

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

    std::vector<int> getDeviceButtonsState()
    {
        std::vector<int> buttons;
        unsigned int buttonCount = 0;
        this->joypadControllerInterface->getButtonCount(buttonCount);

        float value;
        for (unsigned int i = 0; i < buttonCount; i++)
        {
            this->joypadControllerInterface->getButton(i, value);

            if (value > 0)
                buttons.push_back(1);
            else
                buttons.push_back(0);
        }

        return buttons;
    }
};

OpenXRModule::OpenXRModule()
    : m_pImpl{new Impl()} {};

OpenXRModule::~OpenXRModule(){};



bool OpenXRModule::configure(yarp::os::ResourceFinder& rf)
{
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[OpenXRModule::configure] Empty configuration "
                    "for the OpenXRModule application.";
        return false;
    }

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    // get the period
    m_pImpl->dT = generalOptions.check("samplingTime", yarp::os::Value(0.1)).asDouble();
    yInfo() << "[OpenXRModule::configure] sampling time: " << m_pImpl->dT;

    // check if move the robot
    m_pImpl->moveRobot = generalOptions.check("enableMoveRobot", yarp::os::Value(1)).asBool();
    yInfo() << "[OpenXRModule::configure] move the robot: " << m_pImpl->moveRobot;

    // check if move the robot
    m_pImpl->playerOrientationThreshold = generalOptions
                                              .check("playerOrientationThreshold", //
                                                     yarp::os::Value(0.2))
                                              .asDouble();
    yInfo() << "[OpenXRModule::configure] player orientation threshold: "
            << m_pImpl->playerOrientationThreshold;

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    // configure the openxr device
    const std::string headsetGroup = "OPENXR";
    const yarp::os::Bottle& openXROptions = rf.findGroup(headsetGroup);
    if (!m_pImpl->configureOpenXR(openXROptions, getName()))
    {
        yError() << "[OpenXRModule::configure] Unable to configure the oculus";
        return false;
    }

    m_pImpl->head = std::make_unique<HeadRetargeting>();
    yarp::os::Bottle& headOptions = rf.findGroup("HEAD_RETARGETING");
    headOptions.append(generalOptions);
    if (!m_pImpl->head->configure(headOptions, getName()))
    {
        yError() << "[OpenXRModule::configure] Unable to initialize the head retargeting.";
        return false;
    }

    // configure fingers retargeting
    m_pImpl->leftHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& leftFingersOptions = rf.findGroup("LEFT_FINGERS_RETARGETING");
    leftFingersOptions.append(generalOptions);
    if (!m_pImpl->leftHandFingers->configure(leftFingersOptions, getName()))
    {
        yError() << "[OpenXRModule::configure] Unable to initialize the left fingers retargeting.";
        return false;
    }

    m_pImpl->rightHandFingers = std::make_unique<FingersRetargeting>();
    yarp::os::Bottle& rightFingersOptions = rf.findGroup("RIGHT_FINGERS_RETARGETING");
    rightFingersOptions.append(generalOptions);
    if (!m_pImpl->rightHandFingers->configure(rightFingersOptions, getName()))
    {
        yError() << "[OpenXRModule::configure] Unable to initialize the right fingers retargeting.";
        return false;
    }

    // configure hands retargeting
    m_pImpl->leftHand = std::make_unique<HandRetargeting>();
    yarp::os::Bottle& leftHandOptions = rf.findGroup("LEFT_HAND_RETARGETING");
    leftHandOptions.append(generalOptions);
    if (!m_pImpl->leftHand->configure(leftHandOptions))
    {
        yError() << "[OpenXRModule::configure] Unable to initialize the left fingers retargeting.";
        return false;
    }

    m_pImpl->rightHand = std::make_unique<HandRetargeting>();
    yarp::os::Bottle& rightHandOptions = rf.findGroup("RIGHT_HAND_RETARGETING");
    rightHandOptions.append(generalOptions);
    if (!m_pImpl->rightHand->configure(rightHandOptions))
    {
        yError() << "[OpenXRModule::configure] Unable to initialize the right fingers retargeting.";
        return false;
    }

    // open ports
    std::string portName;
    if (!YarpHelper::getStringFromSearchable(rf, "leftHandPosePort", portName))
    {
        yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_pImpl->leftHandPosePort.open("/" + getName() + portName))
    {
        yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "rightHandPosePort", portName))
    {
        yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_pImpl->rightHandPosePort.open("/" + getName() + portName))
    {
        yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
        return false;
    }

    // if (!m_imagesOrientationPort.open("/" + getName() + "/imagesOrientation:o"))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }
    // if (!m_robotOrientationPort.open("/" + getName() + "/robotOrientation:i"))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }

    // if (!YarpHelper::getStringFromSearchable(rf, "playerOrientationPort", portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
    //     return false;
    // }
    // if (!m_playerOrientationPort.open("/" + getName() + portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to open the port " << portName;
    //     return false;
    // }

    if (!YarpHelper::getStringFromSearchable(rf, "rpcWalkingPort_name", portName))
    {
        yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_pImpl->rpcWalkingClient.open("/" + getName() + portName))
    {
        yError() << "[OpenXRModule::configure] " << portName << " port already open.";
        return false;
    }

    // if (!YarpHelper::getStringFromSearchable(rf, "rpcVirtualizerPort_name", portName))
    // {
    //     yError() << "[OpenXRModule::configure] Unable to get a string from a searchable";
    //     return false;
    // }
    // if (!m_rpcVirtualizerClient.open("/" + getName() + portName))
    // {
    //     yError() << "[OpenXRModule::configure] " << portName << " port already open.";
    //     return false;
    // }

    m_pImpl->playerOrientation = 0;
    // m_playerOrientationOld = 0;
    // m_robotYaw = 0;

    m_pImpl->openXRHeadsetPoseInertial.resize(6, 0.0);

    // Reset the cameras if necessary
    bool resetCameras = generalOptions.check("resetCameras", yarp::os::Value(false)).asBool();
    yInfo() << "[OpenXRModule::configure] Reset camera: " << resetCameras;
    if (resetCameras)
    {

        std::string leftCameraPort, rightCameraPort;
        if (!YarpHelper::getStringFromSearchable(generalOptions, "leftCameraPort", leftCameraPort))
        {
            yError() << "[OpenXRModule::configure] resetCameras is true, but leftCameraPort is not "
                        "provided.";
            return false;
        }

        if (!YarpHelper::getStringFromSearchable(
                generalOptions, "rightCameraPort", rightCameraPort))
        {
            yError() << "[OpenXRModule::configure] resetCameras is true, but rightCameraPort is "
                        "not provided.";
            return false;
        }

        if (!m_pImpl->resetCamera(leftCameraPort, "/walking-teleoperation/camera-reset/left"))
        {
            yError() << "[OpenXRModule::configure] Failed to reset left camera.";
            return false;
        }

        if (!m_pImpl->resetCamera(rightCameraPort, "/walking-teleoperation/camera-reset/right"))
        {
            yError() << "[OpenXRModule::configure] Failed to reset right camera.";
            return false;
        }
        yInfo() << "[OpenXRModule::configure] Cameras have been reset.";
    }

    m_pImpl->state = Impl::OpenXRFSM::Configured;

    return true;
}

double OpenXRModule::getPeriod()
{
    return m_pImpl->dT;
}

bool OpenXRModule::close()
{

    m_pImpl->head->controlHelper()->close();

    // if (!m_useSenseGlove)
    // {
    //     m_rightHandFingers->controlHelper()->close();
    //     m_leftHandFingers->controlHelper()->close();
    // }

    m_pImpl->joypadDevice.close();
    m_pImpl->transformClientDevice.close();

    return true;
}


bool OpenXRModule::updateModule()
{
    if (!m_pImpl->getFeedbacks())
    {
        yError() << "[OpenXRModule::updateModule] Unable to get the feedback";
        return false;
    }

    m_pImpl->buttonsState = m_pImpl->getDeviceButtonsState();

    if (m_pImpl->state == Impl::OpenXRFSM::Running)
    {

        // get the transformation form the oculus
        if (!m_pImpl->getTransforms())
        {
            yError() << "[OpenXRModule::updateModule] Unable to get the transform";
            return false;
        }

        m_pImpl->head->setPlayerOrientation(m_pImpl->playerOrientation);
        m_pImpl->head->setDesiredHeadOrientationFromOpenXr(m_pImpl->openXRRoot_T_headOpenXR);

        if (m_pImpl->moveRobot)
        {
            if (!m_pImpl->head->move())
            {
                yError() << "[updateModule::updateModule] unable to move the head";
                return false;
            }
        }

        // update left hand transformation values
        yarp::sig::Vector& leftHandPose = m_pImpl->leftHandPosePort.prepare();
        m_pImpl->leftHand->setPlayerOrientation(m_pImpl->playerOrientation);
        m_pImpl->leftHand->setHandTransform(m_pImpl->openXRRoot_T_lOpenXR);

        // update right hand transformation values
        yarp::sig::Vector& rightHandPose = m_pImpl->rightHandPosePort.prepare();
        m_pImpl->rightHand->setPlayerOrientation(m_pImpl->playerOrientation);
        m_pImpl->rightHand->setHandTransform(m_pImpl->openXRRoot_T_rOpenXR);

        m_pImpl->leftHand->evaluateDesiredHandPose(leftHandPose);
        m_pImpl->rightHand->evaluateDesiredHandPose(rightHandPose);

        // move the robot
        if (m_pImpl->moveRobot)
        {
            m_pImpl->leftHandPosePort.write();
            m_pImpl->rightHandPosePort.write();
        }

        if (m_pImpl->moveRobot)
        {
            // send commands to the walking
            yarp::os::Bottle cmd, outcome;
            double x{0.0}, y{0.0};
            
            if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.joypadButtonsMap))
            {
                m_pImpl->joypadControllerInterface->getAxis(m_pImpl->joypadParameters.xJoypadIndex, x);
                m_pImpl->joypadControllerInterface->getAxis(m_pImpl->joypadParameters.yJoypadIndex, y);

                x = -m_pImpl->joypadParameters.scaleX * m_pImpl->deadzone(x);
                y = m_pImpl->joypadParameters.scaleY * m_pImpl->deadzone(y);
                std::swap(x, y);
            }
            
            // send commands to the walking
            cmd.addString("setGoal");
            cmd.addDouble(x);
            cmd.addDouble(y);
            m_pImpl->rpcWalkingClient.write(cmd, outcome);
        }

        // left fingers
        const double leftFingersVelocity = m_pImpl->evaluateDesiredFingersVelocity(
            m_pImpl->joypadParameters.leftFingersSqueezeButtonsMap,
            m_pImpl->joypadParameters.leftFingersReleaseButtonsMap,
            m_pImpl->joypadParameters.fingersVelocityLeftIndex);

        if (!m_pImpl->leftHandFingers->setFingersVelocity(leftFingersVelocity))
        {
            yError() << "[OpenXRModule::updateModule] Unable to set the left finger velocity.";
            return false;
        }
        if (m_pImpl->moveRobot)
        {
            if (!m_pImpl->leftHandFingers->move())
            {
                yError() << "[OpenXRModule::updateModule] Unable to move the left finger";
                return false;
            }
        }

        const double rightFingersVelocity = m_pImpl->evaluateDesiredFingersVelocity(
            m_pImpl->joypadParameters.rightFingersSqueezeButtonsMap,
            m_pImpl->joypadParameters.rightFingersReleaseButtonsMap,
            m_pImpl->joypadParameters.fingersVelocityRightIndex);

        if (!m_pImpl->rightHandFingers->setFingersVelocity(rightFingersVelocity))
        {
            yError() << "[OpenXRModule::updateModule] Unable to set the right finger velocity.";
            return false;
        }
        if (m_pImpl->moveRobot)
        {
            if (!m_pImpl->rightHandFingers->move())
            {
                yError() << "[OpenXRModule::updateModule] Unable to move the right finger";
                return false;
            }
        }



        // check if it is time to stop walking
        if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.stopWalkingButtonsMap))
        {
            if (m_pImpl->moveRobot)
            {
                yarp::os::Bottle cmd, outcome;
                cmd.addString("stopWalking");
                m_pImpl->rpcWalkingClient.write(cmd, outcome);
            }
            yInfo() << "[OpenXRModule::updateModule] stop";
            return false;
        }

    } else if (m_pImpl->state == Impl::OpenXRFSM::Configured)
    {
        // // check if it is time to prepare or start walking
        if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.prepareWalkingButtonsMap))
        {
            // TODO add a visual feedback for the user
            if (m_pImpl->moveRobot)
            {
                yarp::os::Bottle cmd, outcome;
                cmd.addString("prepareRobot");
                m_pImpl->rpcWalkingClient.write(cmd, outcome);
            }
            m_pImpl->state = OpenXRModule::Impl::OpenXRFSM::InPreparation;
            yInfo() << "[OpenXRModule::updateModule] prepare the robot";
        }
    } else if (m_pImpl->state == Impl::OpenXRFSM::InPreparation)
    {
        if (m_pImpl->moveRobot)
        {
            m_pImpl->head->initializeNeckJointValues();
            if (!m_pImpl->head->move())
            {
                yError() << "[OpenXRModule::updateModule] unable to move the head";
                return false;
            }
        }

        if (m_pImpl->isButtonStateEqualToMask(m_pImpl->joypadParameters.startWalkingButtonsMap))
        {
            if (!m_pImpl->frameTransformInterface->frameExists(m_pImpl->headFrameName))
            {
                yError() << "[OpenXRModule::updateModule] The frame named "
                         << m_pImpl->headFrameName << " does not exist.";
                yError() << "[OpenXRModule::updateModule] I will not start the walking. Please "
                            "try to start again.";
                return true;
            }

            yarp::sig::Matrix openXrHeadInitialTransform = identitySE3();
            if (!m_pImpl->frameTransformInterface->getTransform(
                    m_pImpl->headFrameName, m_pImpl->rootFrameName, openXrHeadInitialTransform))
            {
                yError() << "[OpenXRModule::updateModule] Unable to evaluate the "
                         << m_pImpl->headFrameName << " to " << m_pImpl->rootFrameName
                         << "transformation";
                yError() << "[OpenXRModule::updateModule] I will not start the walking. Please "
                            "try to start again.";
                return true;
            }

            // get only the yaw axis
            iDynTree::Rotation tempRot;
            iDynTree::toEigen(tempRot) = getRotation(openXrHeadInitialTransform);
            double yaw = 0;
            double dummy = 0;
            tempRot.getRPY(dummy, yaw, dummy);

            iDynTree::Transform tempTransform;
            tempTransform.setRotation(iDynTree::Rotation::RotY(
                yaw)); // We remove only the initial rotation of the person head around gravity.
            tempTransform.setPosition(iDynTree::make_span(
                getPosition(openXrHeadInitialTransform))); // We remove the initial position between
                                                           // the head and the reference frame.

            iDynTree::toEigen(m_pImpl->openXRInitialAlignement)
                = iDynTree::toEigen(tempTransform.inverse().asHomogeneousTransform());

            if (m_pImpl->moveRobot)
            {
                yarp::os::Bottle cmd, outcome;
                cmd.addString("startWalking");
                m_pImpl->rpcWalkingClient.write(cmd, outcome);
            }

            // if(outcome.get(0).asBool())
            m_pImpl->state = OpenXRModule::Impl::OpenXRFSM::Running;
            yInfo() << "[OpenXRModule::updateModule] start the robot";
            yInfo() << "[OpenXRModule::updateModule] Running ...";
        }
    }
    return true;
}
