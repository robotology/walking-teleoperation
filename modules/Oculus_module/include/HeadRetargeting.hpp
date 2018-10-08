/**
 * @file HeadRetargeting.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef HEAD_RETARGETING_HPP
#define HEAD_RETARGETING_HPP

// std
#include <memory>

// YARP
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/minJerkCtrl.h>

/**
 * Class useful to manage the head retargeting.
 */
class HeadRetargeting
{
private:

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_headTrajectorySmoother{nullptr};

    double m_playerOrientation;
    yarp::sig::Vector m_desiredHeadOrientation;

public:

    bool configure(const yarp::os::Searchable &config);

    void setPlayerOrientation(const double& playerOrientation);

    void setDesiredHeadOrientation(const yarp::sig::Vector& desiredHeadOrientation);

    void evaluateHeadOrientationCorrected();

    yarp::sig::Vector getHeadOrientation();

};

#endif
