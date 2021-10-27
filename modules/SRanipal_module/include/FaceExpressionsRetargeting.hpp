/**
 * @file FaceExpressionsRetargeting.hpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

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
    double m_lipExpressionThreshold;
    double m_eyeWideSurprisedThreshold;
    std::unordered_map<std::string, std::string> m_currentExpressions;

    void sendFaceExpression(const std::string& part, const std::string& emotion);

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

    void close();

};

#endif // FACEEXPRESSIONSRETARGETING_HPP
