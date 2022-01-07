/**
 * @file Utils.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_UTILS_HPP
#define WALKING_UTILS_HPP

// std
#include <deque>
#include <vector>

// YARP
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>

namespace Angles
{
/**
 * Given 2 angles, it returns the shortest angular difference.
 * The result would always be -pi <= result <= pi.
 *
 * This function is taken from ROS angles
 * [api](http://docs.ros.org/lunar/api/angles/html/namespaceangles.html#a4436fe67ae0c9df020f6779101bbefab).
 * @param fromRad is the starting angle expressed in radians;
 * @param toRad is the final angle expressed in radians.
 * @return the shortest angular distance
 */
double shortestAngularDistance(const double& fromRad, const double& toRad);

/**
 * Normalize angle between  -pi <= angle <= pi.
 * This function is taken from ROS angles
 * [api](http://docs.ros.org/lunar/api/angles/html/namespaceangles.html#a4436fe67ae0c9df020f6779101bbefab).
 * @param angle is the angle
 * @return the wrapped angle
 */
double normalizeAngle(const double& angle);
} // namespace Angles

/**
 * Helper for YARP library.
 */
namespace YarpHelper
{
/**
 * Add a vector of string to a property of a given name.
 * @param prop yarp property;
 * @param key is the key;
 * @param list is the vector of strings that will be added into the property.
 * @return true/false in case of success/failure
 */
bool addVectorOfStringToProperty(yarp::os::Property& prop,
                                 const std::string& key,
                                 const std::vector<std::string>& list);

/**
 * Convert a yarp list into a vector of string
 * @param input is the pointer of a yarp value;
 * @param output is the vector of strings.
 * @return true/false in case of success/failure
 */
bool yarpListToStringVector(yarp::os::Value*& input, std::vector<std::string>& output);

/**
 * Extract a string from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param string is the string.
 * @return true/false in case of success/failure
 */
bool getStringFromSearchable(const yarp::os::Searchable& config,
                             const std::string& key,
                             std::string& string);

/**
 * Extract a double from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param number is the double.
 * @return true/false in case of success/failure
 */
bool getDoubleFromSearchable(const yarp::os::Searchable& config,
                             const std::string& key,
                             double& number);

/**
 * Extract an int from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param number is the int.
 * @return true/false in case of success/failure
 */
bool getIntFromSearchable(const yarp::os::Searchable& config,
                          const std::string& key,
                          int& number);

/**
 * Extract an unsigned int from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param number is the unsigned int.
 * @return true/false in case of success/failure
 */
bool getUnsignedIntFromSearchable(const yarp::os::Searchable& config,
                                  const std::string& key,
                                  unsigned int& number);

/**
 * Extract a boolean from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param value is the boolean.
 * @return true/false in case of success/failure
 */
bool getBooleanFromSearchable(const yarp::os::Searchable& config,
                              const std::string& key,
                              bool& boolean);

/**
 * Extract a vector from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param vector is the vector.
 * @return true/false in case of success/failure
 */
bool getYarpVectorFromSearchable(const yarp::os::Searchable& config,
                                 const std::string& key,
                                 yarp::sig::Vector& vector);

/**
 * Extract an int vector from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param vector is the vector.
 * @return true/false in case of success/failure
 */
bool getIntVectorFromSearchable(const yarp::os::Searchable& config,
                                const std::string& key,
                                std::vector<int>& output);

/**
 * Merge two vectors. vector = [vector, t]
 * @param vector the original vector. The new elements will be add at the end of
 * this vector;
 * @param t vector containing the elements that will be merged with the original
 * vector.
 */
template <typename T> void mergeSigVector(yarp::sig::Vector& vector, const T& t);

/**
 * Variadic fuction used to merge several vectors.
 * @param vector the original vector. The new elements will be add at the end of
 * this vector;
 * @param t vector containing the elements that will be merged with the original
 * vector.
 * @param args list containing all the vector that will be merged.
 */
template <typename T, typename... Args>
void mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args);

/**
 * Send a variadic vector through a yarp buffered port
 * @param port is a Yarp buffered port
 * @param args list containing all the vector that will be send.
 */
template <typename... Args>
void sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args);

/**
 * Add strings to a bottle.
 * @param bottle this bottle will be filled.
 * @param strings list containing all the string.
 */
void populateBottleWithStrings(yarp::os::Bottle& bottle,
                               const std::initializer_list<std::string>& strings);
} // namespace YarpHelper

#include "Utils.tpp"

#endif
