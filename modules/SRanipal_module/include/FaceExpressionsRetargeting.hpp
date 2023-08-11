// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FACEEXPRESSIONSRETARGETING_HPP
#define FACEEXPRESSIONSRETARGETING_HPP

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>
#include <SRanipalInterface.hpp>
#include <unordered_map>
#include <string>

class FaceExpressionsRetargeting
{
    bool m_configured{false};

    yarp::os::RpcClient m_emotionsOutputPort; /** The output port to control the face expressions. **/
    yarp::os::RpcClient m_eyeExpressionsOutputPort; /** The output port to control the eye expressions. **/
    double m_lipExpressionThreshold;
    double m_eyeWideSurprisedThreshold;
    double m_eyeClosedThreshold;
    std::unordered_map<std::string, std::string> m_currentExpressions; /** The key is the part (like "mou" or "leb"), while the value is the current face expression **/
    std::string m_currentEyeExpression;
    bool m_isHappy;

    void sendFaceExpression(const std::string& part, const std::string& emotion);

    void sendEyeExpression(const std::string& emotion);

public:

    FaceExpressionsRetargeting() = default;

    FaceExpressionsRetargeting(const FaceExpressionsRetargeting& other) = delete;

    FaceExpressionsRetargeting(FaceExpressionsRetargeting&& other) = delete;

    FaceExpressionsRetargeting& operator=(const FaceExpressionsRetargeting& other) = delete;

    FaceExpressionsRetargeting& operator=(FaceExpressionsRetargeting&& other) = delete;

    ~FaceExpressionsRetargeting();

    bool configure(yarp::os::ResourceFinder& rf);

    bool updateEyebrows(double eyeWideness);

    bool updateLip(const SRanipalInterface::LipExpressions& lipExpressions);

    bool updateEyeExpressions(double leftEyeOpennes, double rightEyeOpennes);

    void close();

};

#endif // FACEEXPRESSIONSRETARGETING_HPP
