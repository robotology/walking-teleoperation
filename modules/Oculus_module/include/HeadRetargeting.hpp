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

// iDynTree
#include <iDynTree/Core/Rotation.h>

#include <RetargetingHelper.hpp>

/**
 * Class useful to manage the head retargeting.
 */
class HeadRetargeting : public RetargetingHelper
{
private:
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_headTrajectorySmoother{nullptr};

    iDynTree::Rotation m_playerOrientation;
    iDynTree::Rotation m_desiredHeadOrientation;

public:
    /**
     * Configure the object.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    void setPlayerOrientation(const double& playerOrientation);

    void setDesiredHeadOrientation(const yarp::sig::Matrix& desiredHeadOrientation);

    void evaluateHeadOrientationCorrected();
};

#endif
