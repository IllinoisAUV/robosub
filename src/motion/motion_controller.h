#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "robosub/AngularPosition.h"
#include "robosub/AngularVelocity.h"
#include "robosub/LinearPosition.h"
#include "robosub/LinearVelocity.h"

class MotionController {
 public:
  virtual ~MotionController() {}

  // Set roll, pitch and yaw
  virtual void SetRPY(const robosub::AngularPosition rpy) = 0;

  // Set roll, pitch and yaw velocity
  virtual void SetdRPY(const robosub::AngularVelocity drpy) = 0;

  // Set XYZ position
  virtual void SetXYZ(const robosub::LinearPosition xyz) = 0;

  // Set XYZ velocity in body frame
  virtual void SetdXYZ(const robosub::LinearVelocity dxyz) = 0;

  virtual void arming(const std_msgs::Bool arm) = 0;
};

#endif  // MOTION_CONTROLLER_H
