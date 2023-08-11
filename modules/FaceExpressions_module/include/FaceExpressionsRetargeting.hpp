// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FACE_EXPRESSIONS_RETARGETING_H
#define FACE_EXPRESSIONS_RETARGETING_H

#include <fvad.h> /** The external library used to detect the voice. **/
#include <yarp/sig/Sound.h> /** Needed to receive and use the output of the microphone. **/
#include <yarp/os/BufferedPort.h> /** Needed to open the input port. **/
#include <yarp/os/RFModule.h> /** We inherit from this. **/
#include <yarp/os/RpcClient.h> /** Needed to control the face expressions. **/
#include <mutex> /** For mutex and lock_guard. **/

/**
 * @brief The FaceExpressionsRetargetingModule class allows to control the mouth of the robot face expressions if it detects a voice
 * coming from the microphone.
 */
class FaceExpressionsRetargetingModule : public yarp::os::RFModule
{

    Fvad * m_fvadObject {nullptr}; /** The voice activity detection object. **/
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPort; /** The input port for receiving the microphone input. **/
    std::vector<int16_t> m_copiedSound; /** Internal sound buffer. **/
    int m_isTalking{0}; /** Status integer to understand if the operator is talking. **/
    size_t m_switchCounter{0}; /** Internal counter to switch between different mouth positions (to give the impressions it is talking). **/
    bool m_mouthOpen{false}; /** Internal boolean to check if the robot mouth is open or not. **/
    size_t m_switchValue{2}; /** Value after switch the value is moved from open to close. **/
    yarp::os::RpcClient m_emotionsOutputPort; /** The output port to control the face expressions. **/
    double m_period{0.01}; /** The module period. **/
    std::string m_state; /** Mouth state. **/
    std::mutex m_mutex; /** Internal mutex. **/

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


#endif // FACE_EXPRESSIONS_RETARGETING_H
