/**
 * @file HandRetargeting.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef HAND_RETARGETING_HPP
#define HAND_RETARGETING_HPP

// YARP
#include <yarp/os/Bottle.h>

#include <yarp/sig/Matrix.h>

// iDynTree
#include <iDynTree/Core/Transform.h>

/**
 * Class usefull to manage the retargeting of one hand
 */
class HandRetargeting
{
private:
    iDynTree::Transform m_root_T_rootFixedRotated;
    iDynTree::Transform m_rootFixedRotated_T_rootFixed;
    iDynTree::Transform m_rootFixed_T_handOculus;
    double m_scalingFactor;

public:
    bool configure(const yarp::os::Searchable& config);

    void setPlayerOrientation(const double& playerOrientation);

    void setHandTransform(const yarp::sig::Matrix& handTransformation);

    void evaluateHandToRootLinkTransform(yarp::sig::Vector& handPose);
};

#endif
