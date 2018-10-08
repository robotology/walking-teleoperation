/**
 * @file Utils.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iostream>

// YARP
#include <yarp/os/LogStream.h>

template <typename T>
void YarpHelper::mergeSigVector(yarp::sig::Vector& vector, const T& t)
{
    for(int i= 0; i<t.size(); i++)
        vector.push_back(t(i));

    return;
}

template <typename T, typename... Args>
void YarpHelper::mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args)
{
    for(int i= 0; i<t.size(); i++)
        vector.push_back(t(i));

    mergeSigVector(vector, args...);

    return;
}

template <typename... Args>
void YarpHelper::sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args)
{
    yarp::sig::Vector& vector = port.prepare();
    vector.clear();

    mergeSigVector(vector, args...);

    port.write();
}
