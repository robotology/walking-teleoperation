/**
 * @file SRanipalModule.hpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef SRANIPALMODULE_HPP
#define SRANIPALMODULE_HPP

#include <yarp/os/BufferedPort.h> /** Needed to open the input port. **/
#include <yarp/os/RFModule.h> /** We inherit from this. **/
#include <yarp/os/RpcClient.h> /** Needed to control the face expressions. **/
#include <yarp/sig/Image.h> /** To send the lip image. **/
#include <yarp/os/BufferedPort.h> /** To send the lip image. **/
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <mutex> /** For mutex and lock_guard. **/
#include <string> /** For string. **/
#include <unordered_map>

class SRanipalModule : public yarp::os::RFModule
{

    char m_lipImage[800 * 400];
    bool m_useEye;
    bool m_useLip;
    bool m_useRawEyelids;
    double m_period;
    double m_lipExpressionThreshold;
    double m_eyeWideSurprisedThreshold;
    double m_eyeOpenPrecision;
    int m_rawEyelidsCloseValue;
    int m_rawEyelidsOpenValue;
    int m_eyeOpenLevel{-1};
    yarp::os::RpcClient m_emotionsOutputPort; /** The output port to control the face expressions. **/
    yarp::os::BufferedPort<yarp::os::Bottle> m_rawEyelidsOutputPort;
    yarp::os::BufferedPort<yarp::sig::FlexImage> m_lipImagePort;
    yarp::dev::PolyDriver m_poly;
    yarp::dev::IPositionControl* m_iPos{nullptr};
    double m_minEyeLid, m_maxEyeLid;
    std::unordered_map<std::string, std::string> m_currentExpressions;
    std::mutex m_mutex;

    const char * errorCodeToString(int error) const;

    void sendFaceExpression(const std::string& part, const std::string& emotion);

public:

    /**
     * Inherited from RFModule.
     */
    virtual bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Inherited from RFModule.
     */
    virtual double getPeriod() override;

    /**
     * Inherited from RFModule.
     */
    virtual bool updateModule() override;

    /**
     * Inherited from RFModule.
     */
    virtual bool close() override;

};

#endif // SRANIPALMODULE_HPP
