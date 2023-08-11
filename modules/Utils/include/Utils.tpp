// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// std
#include <iostream>

// YARP
#include <yarp/os/LogStream.h>

template <typename T> void YarpHelper::mergeSigVector(yarp::sig::Vector& vector, const T& t)
{
    for (int i = 0; i < t.size(); i++)
        vector.push_back(t(i));

    return;
}

template <typename T, typename... Args>
void YarpHelper::mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args)
{
    for (int i = 0; i < t.size(); i++)
        vector.push_back(t(i));

    mergeSigVector(vector, args...);

    return;
}

template <typename... Args>
void YarpHelper::sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port,
                                    const Args&... args)
{
    yarp::sig::Vector& vector = port.prepare();
    vector.clear();

    mergeSigVector(vector, args...);

    port.write();
}

template <typename T>
bool YarpHelper::checkSizeOfVector(const std::vector<T>& variable,
                                   const size_t& targetSize /*= 0*/,
                                   const char* name /*= ""*/,
                                   const std::string& prefix /*= ""*/)
{
    if (variable.empty())
    {
        yWarning() << prefix << " the variable " << name << " is empty.";
        return true;
    }

    if (variable.size() != targetSize)
    {
        yError() << prefix << "the size of " << std::string(name) << " is not equal to "
                 << targetSize << "." << name << " size: " << variable.size() << ". ";
        return false;
    }
    return true;
}
