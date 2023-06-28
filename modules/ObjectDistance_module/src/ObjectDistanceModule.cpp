/**
 * @file module.cpp
 * @authors
 * @copyright
 */

#include <ObjectDistanceModule.hpp>

#include <iostream>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <yarp/cv/Cv.h>
#include <yarp/sig/Image.h>

#include <opencv2/opencv.hpp>

double ObjectDistanceModule::getPeriod()
{
    return 0.05;
}

bool ObjectDistanceModule::configure(yarp::os::ResourceFinder& rf)
{

    this->setName("ObjectDistanceModule");

    constexpr auto logPrefix = "[ObjectDistanceModule::configure]";

    auto parameterHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    parameterHandler->set(rf);

    auto getParameter
        = [logPrefix](
              std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
              const std::string& paramName,
              auto& param) -> bool {
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            return false;
            BipedalLocomotion::log()->error("{} Invalid parameter handler.", logPrefix);
        }

        if (!ptr->getParameter(paramName, param))
        {
            BipedalLocomotion::log()->error("{} Unable to find the parameter named: {}.", //
                                            logPrefix,
                                            paramName);
            return false;
        }

        return true;
    };

    std::string remoteImagePort, remoteDepthPort, remoteRpcPort;
    if (!getParameter(
            parameterHandler->getGroup("CAMERA_DRIVER"), "remote_image_port", remoteImagePort)
        || !getParameter(
            parameterHandler->getGroup("CAMERA_DRIVER"), "remote_depth_port", remoteDepthPort)
        || !getParameter(
            parameterHandler->getGroup("CAMERA_DRIVER"), "remote_rpc_port", remoteRpcPort))
    {
        return false;
    }

    yarp::os::Property options;
    options.put("device", "RGBDSensorClient");
    options.put("localImagePort", "/" + this->getName() + "/depthCamera/rgbImage:i");
    options.put("localDepthPort", "/" + this->getName() + "/depthCamera/depthImage:i");
    options.put("localRpcPort", "/" + this->getName() + "/depthCamera/rpc:o");
    options.put("ImageCarrier", "mjpeg");
    options.put("DepthCarrier", "tcp+send.portmonitor+file.depthimage_compression_zlib+recv.portmonitor+file.depthimage_compression_zlib+type.dll");
    options.put("remoteImagePort", remoteImagePort);
    options.put("remoteDepthPort", remoteDepthPort);
    options.put("remoteRpcPort", remoteRpcPort);

    if (!m_cameraDevice.open(options))
    {
        BipedalLocomotion::log()->error("{} Unable to open the camera device.", logPrefix);
        return false;
    }

    // initialize the camera bridge
    if (!m_cameraBridge.initialize(parameterHandler->getGroup("CAMERA_BRIDGE")))
    {
        BipedalLocomotion::log()->error("{} Unable to configure the 'Camera bridge'.", logPrefix);
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(&m_cameraDevice, "realsense");
    if (!m_cameraBridge.setDriversList(list))
    {
        BipedalLocomotion::log()->error("{} Unable to set the driver list in the 'Camera bridge'.",
                                        logPrefix);
        return false;
    }

    yarp::os::Time::delay(3);

    m_outputPort.open("/ObjectDistanceModule/distance:o");

    BipedalLocomotion::log()->info("{} Configuration done", logPrefix);

    return true;
}

bool ObjectDistanceModule::updateModule()
{
    constexpr auto logPrefix = "[ObjectDistanceModule::updateModule]";

    if (m_cameraBridge.getMetaData().sensorsList.rgbdCamerasList.size() != 1)
    {
        BipedalLocomotion::log()->error(
            "{} The number of RGBD camera should be equal to 1. Provided {}.",
            logPrefix,
            m_cameraBridge.getMetaData().sensorsList.rgbdCamerasList.size());
        return false;
    }

    for (const std::string& cameraName : m_cameraBridge.getMetaData().sensorsList.rgbdCamerasList)
    {
        // get the image
        cv::Mat frame;
        if (!m_cameraBridge.getDepthImage(cameraName, frame))
        {
            BipedalLocomotion::log()->error(
                "{} Unable to get the image for the camera named: {}.", logPrefix, cameraName);
            return false;
        }

        yarp::sig::ImageOf<yarp::sig::PixelFloat> imgYarp
            = yarp::cv::fromCvMat<yarp::sig::PixelFloat>(frame);

        double average = 0;
        int elements = 0;

        for (int i = frame.rows / 2 - 5; i < frame.rows / 2 + 5; i++)
        {
            for (int j = frame.cols / 2 - 5; j < frame.cols / 2 + 5; j++)
            {
                elements++;
                average += imgYarp(i, j);
            }
        }

        average = average / elements;

        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << average;
        std::string s = stream.str();

        yarp::os::Bottle& data = m_outputPort.prepare();
        data.clear();
        data.addString(s);
        m_outputPort.write();
    }

    return true;
}

bool ObjectDistanceModule::close()
{
    return true;
}