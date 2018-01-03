#ifndef UTIL_H
#define UTIL_H
#include <ros/exception.h>
#include <ros/ros.h>
#include <string>

// Get a parameter. If the parameter is not set, throw an exception
template <typename T>
void getParamOrThrow(std::string param, T &target) throw (ros::Exception) {
  if (!ros::param::get(param, target)) {
    throw ros::Exception("Parameter " + param + " not specified");
  }
}

#endif  // UTIL_H
