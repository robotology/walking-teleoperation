// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LOGGER_HPP
#define LOGGER_HPP

// std
#include <vector>

// teleoperation
#include <Teleoperation.hpp>

// matlogger
#ifdef ENABLE_LOGGER
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif

/**
 * Logger Class useful for logging all the relevant information for the haptic glove teleoperation
 * module.
 */
class HapticGlove::Teleoperation::Logger
{
    bool m_isRightHand; /// <summary> check if the left or right hand
    std::string m_handName; /// <summary> the hand name (.eg., left or right)
    std::string m_logPrefix; /// <summary> logging prefix

    bool m_useSkin; /// <summary> check if using skin

    const Teleoperation&
        m_teleoperation; /// <summary> the constant reference to the parent teleoperation object

    size_t m_numRobotActuatedAxes; /// <summary> the number of robot actuated axis
    size_t m_numRobotActuatedJoints; /// <summary> the number of robot actuated joints

    size_t m_numHumanHandJoints; /// <summary> the number of human hand joints
    size_t m_numHumanHandFingers; /// <summary> the number of human hand finger
    size_t m_numHumanVibrotactileFeedback; /// <summary> the number of vibrotactile feedbacks to the
                                           /// user
    size_t m_numHumanForceFeedback; /// <summary> the number of force feedback to the user
    size_t m_numberRobotTactileFeedbacks; /// <summary> the number of tactile feedback of the robot
                                          /// fingertips

    std::string m_robotPrefix; /// <summary> robot prefix for logging
    std::string m_humanPrefix; /// <summary> human prefix for logging
    std::string m_logFileName; /// <summary> the file name where the data is saved

    Data m_data; /// <summary> the data structure

#ifdef ENABLE_LOGGER
    XBot::MatLogger2::Ptr m_logger; /**< the pointer to the logger */
    XBot::MatAppender::Ptr m_appender;
#endif

    /**
     * update the data structure of the logger
     * */
    bool updateData();

public:
    /**
     * Constructor
     * @param module a constant reference to the parent teleoperation object
     * @param isRightHand check if the right hand or the left hand
     */
    Logger(const Teleoperation& module, const bool isRightHand);

    /**
     * Destructor
     * */
    ~Logger();

    /**
     * open the logger
     * */
    bool openLogger();

    /**
     * log the data
     * */
    bool logData();

    /**
     * close the logger
     * */
    bool closeLogger();
};

#endif // LOGGER_HPP
