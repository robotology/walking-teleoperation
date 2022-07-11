/**
 * @file FaceExpressionsRetargeting.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <FaceExpressionsRetargeting.hpp>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>

void FaceExpressionsRetargeting::sendFaceExpression(const std::string &part, const std::string &emotion)
{
    // See https://robotology.github.io/robotology-documentation/doc/html/group__icub__faceExpressions.html
    //for the set of expressions and messages currently supported
    if (emotion != m_currentExpressions[part])
    {
        yarp::os::Bottle cmd, reply;
        cmd.addVocab32(yarp::os::Vocab32::encode("set"));
        cmd.addVocab32(yarp::os::Vocab32::encode(part));
        cmd.addVocab32(yarp::os::Vocab32::encode(emotion));
        m_emotionsOutputPort.write(cmd, reply);
        m_currentExpressions[part] = emotion;
        yInfo() << "[FaceExpressionsRetargeting::sendFaceExpression] Sending" << emotion << "to" << part;
    }
}

FaceExpressionsRetargeting::~FaceExpressionsRetargeting()
{
    close();
}

bool FaceExpressionsRetargeting::configure(yarp::os::ResourceFinder &rf)
{
    if (m_configured)
    {
        yError() << "[FaceExpressionsRetargeting::configure] The face expressions retargeting is already configured.";
        return false;
    }

    std::string name = rf.check("name", yarp::os::Value("SRanipalModule"), "The name of the module.").asString();

    std::string emotionsPortOut = rf.check("emotionsOutputPortName", yarp::os::Value("/emotions:o"), "The name of the output port for the emotions.").asString();
    if (!m_emotionsOutputPort.open("/" + name + emotionsPortOut))
    {
        yError() << "[SRanipalModule::configure] Failed to open /" + name + emotionsPortOut + " port.";
        return false;
    }

    m_lipExpressionThreshold = rf.check("lipExpressionThreshold", yarp::os::Value(0.2)).asFloat64();
    m_eyeWideSurprisedThreshold = rf.check("eyeWideSurprisedThreshold", yarp::os::Value(0.2)).asFloat64();

    m_configured = true;

    return true;
}

bool FaceExpressionsRetargeting::updateEyebrows(double eyeWideness)
{
    if (!m_configured)
    {
        yError() << "[FaceExpressionsRetargeting::updateEyebrows] The face expressions retargeting is not configured.";
        return false;
    }

    std::string leftEyeBrow = "neu";
    std::string rightEyeBrow = "neu";

    if (eyeWideness > m_eyeWideSurprisedThreshold)
    {
        leftEyeBrow = "sur";
        rightEyeBrow = "sur";
    }

    sendFaceExpression("leb", leftEyeBrow);
    sendFaceExpression("reb", rightEyeBrow);

    return true;
}

bool FaceExpressionsRetargeting::updateLip(const SRanipalInterface::LipExpressions &lipExpressions)
{
    if (!m_configured)
    {
        yError() << "[FaceExpressionsRetargeting::updateLip] The face expressions retargeting is not configured.";
        return false;
    }

    std::string mouthExpression = "neu";
    if (lipExpressions.mouthOpen > m_lipExpressionThreshold)
    {
        mouthExpression = "sur";
    }
    else if (lipExpressions.smile > m_lipExpressionThreshold)
    {
        mouthExpression = "hap";
    }
    else if (lipExpressions.sad > m_lipExpressionThreshold)
    {
        mouthExpression = "sad";
    }

    sendFaceExpression("mou", mouthExpression);

    return true;
}

void FaceExpressionsRetargeting::close()
{
    m_emotionsOutputPort.close();
    m_configured = false;
}
