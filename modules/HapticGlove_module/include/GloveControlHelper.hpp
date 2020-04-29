/**
 * @file GloveControlHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef GLOVE_CONTROL_HELPER_HPP
#define GLOVE_CONTROL_HELPER_HPP

// std
#include <memory>

// YARP

#include <yarp/sig/Vector.h>

// Sense Glove
#include "DeviceList.h"
#include "SenseGlove.h"
/**
 * GloveControlHelper is an helper class for controlling the glove.
 */
namespace HapticGlove
{

class GloveControlHelper
{

    int m_forceFbDof; /**< Number of the actuated motors Dofs to produce force feedback to the
                         human*/
    int m_buzzDof; /**< Number of the actuated vibro tactile Dofs to produce vibro tactile
                         feedback to the human*/
    int m_jointsDof; /**< Number of the joints of the hand model*/

    bool m_isReady; /**< true if the glove is ready to use, communication working*/

    bool m_isRightHand; /**< true if the glove is the right hand*/


    std::vector<int> m_desiredForceValues; /**< Desired joint value [deg or deg/s]. */
    std::vector<int> m_desiredBuzzValues; /**< Joint position [deg]. */
    yarp::sig::Vector m_jointsFeedbackInRadians; /**< Joint position [rad]. */

    SGCore::SG::SenseGlove m_glove;

public:
    /**
     * Configure the helper
     * @param config confifuration options
     * @param name name of the robot
     * @param isMandatory if true the helper will return an error if there is a
     * problem in the configuration phase
     * @return true / false in case of success / failure
     */
    bool
    configure(const yarp::os::Searchable& config, const std::string& name, const bool& rightHand);

    /**
     * Set the desired joint reference (position or velocity)
     * @param desiredValue desired joint velocity or position (radiant or radiant/s)
     * @return true / false in case of success / failure
     */
    bool setFingersForceReference(const yarp::sig::Vector& desiredValue);

    /**
     * Get the measured joints force
     * @param measuredValue measured joint force
     * @return true / false in case of success / failure
     */
    bool getFingersForceMeasured(yarp::sig::Vector& measuredValue);

    /**
     * Get the measured joint values
     * @param measuredValue measured joint values
     * @return true / false in case of success / failure
     */
    bool getFingersJointsMeasured(yarp::sig::Vector& measuredValue);

    /**
     * Set the number of vibro-tactile reference
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool setBuzzMotorsReference(const yarp::sig::Vector& desiredValue);


    bool setPalmFeedback(const int desiredValue);
    
    /**
     * Set the number of vibro-tactile reference
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool turnOffBuzzMotors();


    int getNoOfBuzzMotors();

    /**
     * Setup the communication with the glove
     * @return true / false in case of success / failure
     */
    bool setupGlove();

    bool stopFeedback();

    





    /**
     * Close the helper
     */
    void close();
};
} // namespace HapticGlove

#endif
