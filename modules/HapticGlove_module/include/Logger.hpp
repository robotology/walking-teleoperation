// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LOGGER_HPP
#define LOGGER_HPP

// std
#include <vector>

// teleoperation
#include <Teleoperation.hpp>

// blf
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

struct LoggerOptions
{
    bool isRightHand = false; /// <summary> check if the right hand
    bool dumpSkinData = false; /// <summary> check if dumping skin
    bool dumpHumanData = false; /// <summary> check if dumping human data
    bool dumpKalmanFilterData = false; /// <summary> check if dumping kalman filter data
};

/**
 * Logger Class useful for logging all the relevant information for the haptic glove teleoperation
 * module.
 */
class HapticGlove::Teleoperation::Logger
{
    std::string m_handName; /// <summary> the hand name (.eg., left or right)

    LoggerOptions m_options; /// <summary> the logger options

    std::vector<double> m_fingerSkinContactBuffer; /// <summary> A buffer for the finger skin contact to convert from bool

    const Teleoperation&
        m_teleoperation; /// <summary> the constant reference to the parent teleoperation object

    BipedalLocomotion::YarpUtilities::VectorsCollectionServer& m_logger; /**< the pointer to the logger */

public:
    /**
     * Constructor
     * @param module a constant reference to the parent teleoperation object
     * @param isRightHand check if the right hand or the left hand
     */
    Logger(const Teleoperation& module,
           LoggerOptions options,
           BipedalLocomotion::YarpUtilities::VectorsCollectionServer& loggerObject);

    /**
     * Destructor
     * */
    ~Logger() = default;

    /**
     * log the data
     * */
    bool logData();

};

#endif // LOGGER_HPP
