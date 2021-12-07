/**
 * @file Utils.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#define _USE_MATH_DEFINES
#include <cmath>

// YARP
#include <yarp/os/LogStream.h>

#include "Utils.hpp"

bool YarpHelper::addVectorOfStringToProperty(yarp::os::Property& prop,
                                             const std::string& key,
                                             const std::vector<std::string>& list)
{
    // check if the key already exists
    if (prop.check(key))
    {
        yError() << "[addVectorOfStringToProperty] The property already exist.";
        return false;
    }

    prop.addGroup(key);
    yarp::os::Bottle& bot = prop.findGroup(key).addList();
    for (size_t i = 0; i < list.size(); i++)
        bot.addString(list[i].c_str());

    return true;
}

bool YarpHelper::yarpListToStringVector(yarp::os::Value*& input, std::vector<std::string>& output)
{
    // clear the std::vector
    output.clear();

    // check if the yarp value is a list
    if (!input->isList())
    {
        yError() << "[yarpListToStringVector] The input is not a list.";
        return false;
    }

    yarp::os::Bottle* bottle = input->asList();
    for (int i = 0; i < bottle->size(); i++)
    {
        // check if the elements of the bottle are strings
        if (!bottle->get(i).isString())
        {
            yError() << "[yarpListToStringVector] There is a field that is not a string.";
            return false;
        }
        output.push_back(bottle->get(i).asString());
    }
    return true;
}

bool YarpHelper::getStringFromSearchable(const yarp::os::Searchable& config,
                                         const std::string& key,
                                         std::string& string)
{
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        yError() << "[getStringFromSearchable] Missing field " << key;
        return false;
    }

    if (!value->isString())
    {
        yError() << "[getStringFromSearchable] the value is not a string.";
        return false;
    }

    string = value->asString();
    return true;
}

bool YarpHelper::getDoubleFromSearchable(const yarp::os::Searchable& config,
                                         const std::string& key,
                                         double& number)
{
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        yError() << "[getNumberFromSearchable] Missing field " << key;
        return false;
    }

    if (!value->isDouble())
    {
        yError() << "[getNumberFromSearchable] the value is not a double.";
        return false;
    }

    number = value->asDouble();
    return true;
}

bool YarpHelper::getYarpVectorFromSearchable(const yarp::os::Searchable& config,
                                             const std::string& key,
                                             yarp::sig::Vector& output)
{
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        yError() << "[getYarpVectorFromSearchable] Missing field " << key;
        return false;
    }

    if (!value->isList())
    {
        yError() << "[getYarpVectorFromSearchable] the value is not a double.";
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();

    if (inputPtr->size() != output.size())
    {
        yError() << "[getYarpVectorFromSearchable] The size of the YARP vector and "
                    "the size of "
                 << "the YARP list are not coherent.";
        return false;
    }

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isDouble() && !inputPtr->get(i).isInt())
        {
            yError() << "[getYarpVectorFromSearchable] The input is expected to be a "
                        "double or a int";
            return false;
        }
        output(i) = inputPtr->get(i).asDouble();
    }
    return true;
}

bool YarpHelper::getVectorFromSearchable(const yarp::os::Searchable& config,
                                         const std::string& key,
                                         std::vector<double>& output)
{
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        yError() << "[getYarpVectorFromSearchable] Missing field " << key;
        return false;
    }

    if (!value->isList())
    {
        yError() << "[getYarpVectorFromSearchable] the value is not a double.";
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();

    output.resize(inputPtr->size(), 0.0);

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isDouble() && !inputPtr->get(i).isInt())
        {
            yError() << "[getYarpVectorFromSearchable] The input is expected to be a "
                        "double or a int";
            return false;
        }
        output[i] = inputPtr->get(i).asDouble();
    }
    return true;
}

bool YarpHelper::getVectorFromSearchable(const yarp::os::Searchable& config,
                                         const std::string& key,
                                         std::vector<std::string>& output)
{
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        yError() << "[getYarpVectorFromSearchable] Missing field " << key;
        return false;
    }

    if (!value->isList())
    {
        yError() << "[getYarpVectorFromSearchable] the value is not a list.";
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();

    output.resize(inputPtr->size());

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isString())
        {
            yError() << "[getYarpVectorFromSearchable] The input is expected to be a "
                        "string";
            return false;
        }
        output[i] = inputPtr->get(i).asString();
    }
    return true;
}

void YarpHelper::populateBottleWithStrings(yarp::os::Bottle& bottle,
                                           const std::initializer_list<std::string>& strings)
{
    for (const auto& string : strings)
        bottle.addString(string);
}

double normalizeAnglePositive(const double& angle)
{
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double Angles::normalizeAngle(const double& angle)
{
    double a = normalizeAnglePositive(angle);
    if (a > M_PI)
        a -= 2.0 * M_PI;
    return a;
}

double Angles::shortestAngularDistance(const double& fromRad, const double& toRad)
{
    return normalizeAngle(toRad - fromRad);
}

bool YarpHelper::getIntFromSearchable(const yarp::os::Searchable& config,
                                      const std::string& key,
                                      int& number)
{
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        yError() << "[getIntFromSearchable] Missing field " << key;
        return false;
    }

    if (!value->isInt())
    {
        yError() << "[getIntFromSearchable] the value is not an int.";
        return false;
    }

    number = value->asInt();
    return true;
}

bool YarpHelper::getUnsignedIntFromSearchable(const yarp::os::Searchable& config,
                                              const std::string& key,
                                              unsigned int& number)
{
    int value;
    if (!YarpHelper::getIntFromSearchable(config, key, value))
    {
        return false;
    }

    if (value < 0)
    {
        yError() << "[getUnsignedIntFromSearchable] the value is lower than zero.";
        return false;
    }

    number = static_cast<unsigned int>(value);

    return true;
}

bool YarpHelper::getBooleanFromSearchable(const yarp::os::Searchable& config,
                                          const std::string& key,
                                          bool& boolean)
{
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        yError() << "[getBooleanFromSearchable] Missing field " << key;
        return false;
    }

    if (!value->isBool())
    {
        yError() << "[getBooleanFromSearchable] the value is not a bool.";
        return false;
    }

    boolean = value->asBool();
    return true;
}

std::string YarpHelper::getTimeDateMatExtension()
{
    // this code snippet is taken from
    // https://stackoverflow.com/questions/17223096/outputting-date-and-time-in-c-using-stdchrono
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char timedate[30];

    std::strftime(&timedate[0], 30, "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    std::string timeDateStr = timedate;
    timeDateStr.shrink_to_fit();
    return timeDateStr;
}
