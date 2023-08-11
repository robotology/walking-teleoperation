// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <FaceExpressionsRetargeting.hpp>
#include <yarp/os/Vocab.h>
#include <yarp/os/LogStream.h>
#include <functional>
#include <cmath>

#define VAD_FREQUENCY     8000 //Other frequencies do not seem to work well
#define VAD_SAMPLE_LENGTH 80 //This seeemed to be the only value for which there was a usable output.

bool FaceExpressionsRetargetingModule::configure(yarp::os::ResourceFinder &rf)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    yInfo() << "Configuring";
    m_fvadObject = fvad_new();

    if (!m_fvadObject)
    {
        yError() << "Failed to created VAD object";
        return false;
    }

    std::string name = rf.check("name", yarp::os::Value("face_expressions_from_mic"), "The name of the module.").asString();
    setName(name.c_str());

    std::string audioPortIn = rf.check("audio_input_port_name", yarp::os::Value("/in"), "The name of the input port for the audio.").asString();
    m_audioPort.open("/" + name + audioPortIn);

    std::string emotionsPortOut = rf.check("emotions_output_port_name", yarp::os::Value("/emotions:o"), "The name of the output port for the emotions.").asString();

    m_period = rf.check("period", yarp::os::Value(0.01), "The module period.").asFloat64();

    m_switchValue = rf.check("expression_counter_max_value", yarp::os::Value(2), "It controls the frequency with which the robot opens and close the mouth.").asInt32();

    m_emotionsOutputPort.open("/" + name + emotionsPortOut);

    /*
         * Changes the VAD operating ("aggressiveness") mode of a VAD instance.
         *
         * A more aggressive (higher mode) VAD is more restrictive in reporting speech.
         * Put in other words the probability of being speech when the VAD returns 1 is
         * increased with increasing mode. As a consequence also the missed detection
         * rate goes up.
         *
         * Valid modes are 0 ("quality"), 1 ("low bitrate"), 2 ("aggressive"), and 3
         * ("very aggressive"). The default mode is 0.
         *
         * Returns 0 on success, or -1 if the specified mode is invalid.
         */
    fvad_set_mode(m_fvadObject, 3); //Using 3 as the other methods are indeed too sensitive for our application.

    /*
         * Sets the input sample rate in Hz for a VAD instance.
         *
         * Valid values are 8000, 16000, 32000 and 48000. The default is 8000. Note
         * that internally all processing will be done 8000 Hz; input data in higher
         * sample rates will just be downsampled first.
         *
         * Returns 0 on success, or -1 if the passed value is invalid.
         */
    if (fvad_set_sample_rate(m_fvadObject, VAD_FREQUENCY))
    {
        yError() << "Unsupported input frequency.";
        return false;
    }

    m_copiedSound.resize(VAD_SAMPLE_LENGTH);

    yInfo() << "Started";


    return true;
}

double FaceExpressionsRetargetingModule::getPeriod()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_period;
}

bool FaceExpressionsRetargetingModule::updateModule()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_audioPort.getPendingReads())
    {
        yarp::sig::Sound* inputSound = m_audioPort.read(false);

        if (inputSound->getFrequency() < VAD_FREQUENCY)
        {
            yError() << "The frequency needs to be at least " << VAD_FREQUENCY;
            return false;
        }

        int subsampling = std::round(inputSound->getFrequency() / (double)VAD_FREQUENCY);

        size_t soundLength = VAD_SAMPLE_LENGTH * subsampling;

        if (soundLength > inputSound->getSamples())
        {
            yError() << "The input sound is too short.";
            return false;
        }

        for (size_t i = 0; i < VAD_SAMPLE_LENGTH; ++i)
        {
            m_copiedSound[i] = inputSound->get(i*subsampling);
        }

        /*
         * Calculates a VAD decision for an audio frame.
         *
         * `frame` is an array of `length` signed 16-bit samples. Only frames with a
         * length of 10, 20 or 30 ms are supported, so for example at 8 kHz, `length`
         * must be either 80, 160 or 240.
         *
         * Returns              : 1 - (active voice),
         *                        0 - (non-active Voice),
         *                       -1 - (invalid frame length).
         */
        m_isTalking = fvad_process(m_fvadObject, m_copiedSound.data(), m_copiedSound.size());

        if (m_isTalking < 0)
        {
            yError() << "Invalid frame length.";
            return false;
        }

        if (m_isTalking)
        {
            m_switchCounter++;

            if (m_switchCounter > m_switchValue)
            {
                m_mouthOpen = !m_mouthOpen;
                m_switchCounter = 0;
            }
        }
        else
        {
            m_mouthOpen = false;
            m_switchCounter = 0;
        }

        if (m_mouthOpen)
            m_state="sur";
        else
            m_state="neu";

        yarp::os::Bottle cmd, reply;
        cmd.addVocab32(yarp::os::Vocab32::encode("set"));
        cmd.addVocab32(yarp::os::Vocab32::encode("mou"));
        cmd.addVocab32(yarp::os::Vocab32::encode(m_state));
        m_emotionsOutputPort.write(cmd,reply);

    }

    return true;
}

bool FaceExpressionsRetargetingModule::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    fvad_free(m_fvadObject);
    m_fvadObject = nullptr;
    m_audioPort.close();
    m_emotionsOutputPort.close();
    yInfo() << "Closing";

    return true;
}
