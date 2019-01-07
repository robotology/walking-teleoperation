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

#include <RetargetingController.hpp>

/**
 * Class useful to manage the head retargeting.
 */
class HeadRetargeting : public RetargetingController
{
private:
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_headTrajectorySmoother{nullptr};

    iDynTree::Rotation m_playerOrientation;
    iDynTree::Rotation m_desiredHeadOrientation;
    iDynTree::Rotation m_oculusRoot_T_oculusHeadset;

    // double m_playerOrientation;
    // yarp::sig::Vector m_desiredHeadOrientation;

public:
    /**
     * Configure the object.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configure(const yarp::os::Searchable& config, const std::string& name) override;

    void setPlayerOrientation(const double& playerOrientation);

    void setDesiredHeadOrientation(const yarp::sig::Matrix& desiredHeadOrientation);

    void setDesiredHeadOrientation(const yarp::sig::Vector& desiredHeadOrientation);

    iDynTree::Vector3 inverseKinematics(const iDynTree::Rotation& matrix);

    iDynTree::Rotation forwardKinematics(const yarp::sig::Vector& YXZ);

    bool move() override;
};

#endif
