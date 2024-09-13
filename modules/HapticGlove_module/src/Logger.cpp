// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// teleoperation
#include <Logger.hpp>
#include <Utils.hpp>

// yarp
#include <yarp/os/LogStream.h>

using namespace HapticGlove;
HapticGlove::Teleoperation::Logger::Logger(
    const Teleoperation& module,
    LoggerOptions options,
    BipedalLocomotion::YarpUtilities::VectorsCollectionServer& loggerObject)
    : m_handName(options.isRightHand ? "right_hand" : "left_hand")
    , m_options(options)
    , m_teleoperation(module)
    , m_logger(loggerObject)
{
    // axis
    m_logger.populateMetadata(m_handName + "::actuated_axes::measured",
                              m_teleoperation.getActuatedAxisNames());
    m_logger.populateMetadata(m_handName + "::actuated_axes::desired",
                              m_teleoperation.getActuatedAxisNames());
    m_logger.populateMetadata(m_handName + "::actuated_axes::pid_output",
                              m_teleoperation.getActuatedAxisNames());

    m_logger.populateMetadata(m_handName + "::actuated_joints::measured",
                              m_teleoperation.getActuatedJointNames());
    m_logger.populateMetadata(m_handName + "::actuated_joints::desired",
                              m_teleoperation.getActuatedJointNames());

    if (m_options.dumpKalmanFilterData)
    {
        // axis reference KF
        m_logger.populateMetadata(m_handName
                                  + "::kalman_filter::actuated_axes::position::desired",
                                  m_teleoperation.getActuatedAxisNames());
        m_logger.populateMetadata(m_handName + "::kalman_filter::actuated_axes::velocity::desired",
                                  m_teleoperation.getActuatedAxisNames());
        m_logger.populateMetadata(m_handName + "::kalman_filter::actuated_axes::acceleration::desired",
                                  m_teleoperation.getActuatedAxisNames());

        // axis feedback KF
        m_logger.populateMetadata(m_handName + "::kalman_filter::actuated_axes::position::measured",
                                  m_teleoperation.getActuatedAxisNames());
        m_logger.populateMetadata(m_handName + "::kalman_filter::actuated_axes::velocity::measured",
                                  m_teleoperation.getActuatedAxisNames());
        m_logger.populateMetadata(m_handName + "::kalman_filter::actuated_axes::acceleration::measured",
                                  m_teleoperation.getActuatedAxisNames());

        // joints KF
        m_logger.populateMetadata(m_handName + "::kalman_filter::actuated_joints::position::measured",
                                  m_teleoperation.getActuatedJointNames());
        m_logger.populateMetadata(m_handName + "::kalman_filter::actuated_joints::position::desired",
                                  m_teleoperation.getActuatedJointNames());
    }

    if (m_options.dumpHumanData)
    {
        // Human data
        m_logger.populateMetadata(m_handName + "::human::joint_values",
                                  m_teleoperation.getHumanHandJointsNames());
        m_logger.populateMetadata(m_handName + "::human::force_feedbacks",
                                  m_teleoperation.getHumanHandFingerNames());
        m_logger.populateMetadata(m_handName + "::human::vibrotactile_feedbacks",
                                  m_teleoperation.getHumanHandFingerNames());
    }

    // skin data
    if (m_options.dumpSkinData)
    {
        const std::vector<FingertipTactileData>& tactileData
            = m_teleoperation.getFingersTactileData();

        std::vector<std::string> fingerNames;
        for (const auto& data : tactileData)
        {
            std::vector<std::string> taxelNames;
            for (size_t i = data.indexStart; i <= data.indexEnd; i++)
            {
                taxelNames.push_back("taxel_" + std::to_string(i));
            }

            m_logger.populateMetadata(m_handName + "::skin::" + data.fingerName + "::taxels::raw",
                                      taxelNames);
            m_logger.populateMetadata(m_handName + "::skin::" + data.fingerName + "::taxels::calibrated",
                                      taxelNames);
            m_logger.populateMetadata(m_handName + "::skin::" + data.fingerName + "::taxels::calibrated_derivative",
                                      taxelNames);
            fingerNames.push_back(data.fingerName);
        }
        m_logger.populateMetadata(m_handName + "::skin::contact_strength::value", fingerNames);
        m_logger.populateMetadata(m_handName + "::skin::contact_strength::derivative", fingerNames);
        m_logger.populateMetadata(m_handName + "::skin::vibrotactile_feedback::absolute_value", fingerNames);
        m_logger.populateMetadata(m_handName + "::skin::vibrotactile_feedback::derivative_value", fingerNames);
        m_logger.populateMetadata(m_handName + "::skin::vibrotactile_feedback::total_value", fingerNames);
        m_logger.populateMetadata(m_handName + "::skin::is_in_contact", fingerNames);
        m_fingerSkinContactBuffer.resize(fingerNames.size());
    }
}

bool Teleoperation::Logger::logData()
{
    const auto& data = m_teleoperation.getData();
    m_logger.populateData(m_handName + "::actuated_axes::measured", data.robotAxisFeedbacks);
    m_logger.populateData(m_handName + "::actuated_axes::desired", data.robotAxisReferences);
    m_logger.populateData(m_handName + "::actuated_axes::pid_output", data.robotMotorPidOutputs);

    m_logger.populateData(m_handName + "::actuated_joints::measured", data.robotJointFeedbacks);
    m_logger.populateData(m_handName + "::actuated_joints::desired",data.robotJointReferences);

    if (m_options.dumpKalmanFilterData)
    {
        // axis reference KF
        m_logger.populateData(m_handName + "::kalman_filter::actuated_axes::position::desired",
                              data.robotAxisValueReferencesKf);
        m_logger.populateData(m_handName + "::kalman_filter::actuated_axes::velocity::desired",
                              data.robotAxisVelocityReferencesKf);
        m_logger.populateData(m_handName + "::kalman_filter::actuated_axes::acceleration::desired",
                              data.robotAxisAccelerationReferencesKf);

        // axis feedback KF
        m_logger.populateData(m_handName + "::kalman_filter::actuated_axes::position::measured",
                              data.robotAxisValueFeedbacksKf);
        m_logger.populateData(m_handName + "::kalman_filter::actuated_axes::velocity::measured",
                              data.robotAxisVelocityFeedbacksKf);
        m_logger.populateData(m_handName + "::kalman_filter::actuated_axes::acceleration::measured",
                              data.robotAxisAccelerationFeedbacksKf);

        // joints KF
        m_logger.populateData(m_handName + "::kalman_filter::actuated_joints::position::measured",
                              data.robotJointsFeedbackKf);
        m_logger.populateData(m_handName + "::kalman_filter::actuated_joints::position::desired",
                              data.robotJointsExpectedKf);
    }

    if (m_options.dumpHumanData)
    {
        // Human data
        m_logger.populateData(m_handName + "::human::joint_values",
                              data.humanJointValues);
        m_logger.populateData(m_handName + "::human::force_feedbacks",
                              data.humanForceFeedbacks);
        m_logger.populateData(m_handName + "::human::vibrotactile_feedbacks",
                              data.humanVibrotactileFeedbacks);
    }

    // skin data
    if (m_options.dumpSkinData)
    {
        const std::vector<FingertipTactileData>& tactileData
            = m_teleoperation.getFingersTactileData();

        for (const auto& finger : tactileData)
        {
            m_logger.populateData(m_handName + "::skin::" + finger.fingerName + "::taxels::raw",
                                  iDynTree::Span(data.fingertipsSkinData.data() + finger.indexStart, finger.noTactileSensors));
            m_logger.populateData(m_handName + "::skin::" + finger.fingerName + "::taxels::calibrated",
                                  iDynTree::Span(data.fingertipsCalibratedTactileFeedback.data() + finger.indexStart,
                                                 finger.noTactileSensors));
            m_logger.populateData(m_handName + "::skin::" + finger.fingerName + "::taxels::calibrated_derivative",
                                  iDynTree::Span(data.fingertipsCalibratedDerivativeTactileFeedback.data()
                                                 + finger.indexStart,
                                                 finger.noTactileSensors));
        }
        m_logger.populateData(m_handName + "::skin::contact_strength::value",
                              data.fingercontactStrengthFeedback);
        m_logger.populateData(m_handName + "::skin::contact_strength::derivative",
                              data.fingercontactStrengthDerivativeFeedback);
        m_logger.populateData(m_handName + "::skin::vibrotactile_feedback::absolute_value",
                              data.robotFingerSkinAbsoluteValueVibrotactileFeedbacks);
        m_logger.populateData(m_handName + "::skin::vibrotactile_feedback::derivative_value",
                              data.robotFingerSkinDerivativeValueVibrotactileFeedbacks);
        m_logger.populateData(m_handName + "::skin::vibrotactile_feedback::total_value",
                              data.robotFingerSkinTotalValueVibrotactileFeedbacks);
        for (size_t i = 0; i < data.areFingersSkinInContact.size(); i++)
        {
            m_fingerSkinContactBuffer[i] = data.areFingersSkinInContact[i] ? 1.0 : 0.0;
        }
        m_logger.populateData(m_handName + "::skin::is_in_contact", m_fingerSkinContactBuffer);
    }

    return true;
}
