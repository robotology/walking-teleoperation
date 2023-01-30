/**
 * @file SRanipalModule.hpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef SRANIPALMODULE_HPP
#define SRANIPALMODULE_HPP

#include <SRanipalInterface.hpp>
#include <EyelidsRetargeting.hpp>
#include <FaceExpressionsRetargeting.hpp>
#include <GazeRetargeting.hpp>
#include <AdvancedJoypad.hpp>
#include <VRInterface.hpp>
#include <yarp/os/BufferedPort.h> /** Needed to open the input port. **/
#include <yarp/os/RFModule.h> /** We inherit from this. **/
#include <yarp/sig/Image.h> /** To send the lip image. **/
#include <yarp/os/BufferedPort.h> /** To send the lip image. **/
#include <mutex> /** For mutex and lock_guard. **/
#include <string> /** For string. **/
#include <unordered_map>
#include <memory>

class SRanipalModule : public yarp::os::RFModule
{

    SRanipalInterface m_sranipalInterface;
    EyelidsRetargeting m_eyelidsRetargeting;
    FaceExpressionsRetargeting m_faceExpressions;
    GazeRetargeting m_gazeRetargeting;
    AdvancedJoypad m_advancedJoypad;

    std::shared_ptr<VRInterface> m_VRInterface{nullptr};

    bool m_useEyebrows;
    bool m_useLip;
    bool m_useEyelids;
    bool m_useGaze;
    bool m_useAdvancedJoypad;

    double m_period;
    yarp::os::BufferedPort<yarp::sig::FlexImage> m_lipImagePort;
    std::mutex m_mutex;

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
