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
#include <mutex> /** For mutex and lock_guard. **/
#include <string> /** For string. **/

class SRanipalModule : public yarp::os::RFModule
{

    char m_lipImage[800 * 400];
    bool m_useEye;
    bool m_useLip;
    double m_period;
    double m_lipExpressionThreshold;
    yarp::os::RpcClient m_emotionsOutputPort; /** The output port to control the face expressions. **/
    yarp::os::BufferedPort<yarp::sig::FlexImage> m_lipImagePort;
    std::mutex m_mutex;

    const char * errorCodeToString(int error) const;

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
