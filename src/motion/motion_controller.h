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
  MotionController();
  virtual ~MotionController() {}

  void Start();

  // Set roll, pitch and yaw
  void SetRPY(const robosub::AngularPosition rpy);

  // Set roll, pitch and yaw velocity
  void SetdRPY(const robosub::AngularVelocity drpy);

  // Set XYZ position
  void SetXYZ(const robosub::LinearPosition xyz);

  // Set XYZ velocity in body frame
  void SetdXYZ(const robosub::LinearVelocity dxyz);

  // Arm or disarm the sub
  void Arming(const std_msgs::Bool arm);

  // Run an iteration of the motion controller
  void Update(const ros::TimerEvent &event);

 protected:
  virtual void DoUpdate() = 0;
  virtual void DoArming(bool arm) = 0;

  // Period with which Update() is called
  constexpr static float kPeriod = 0.1;

  // Velocity timeouts
  void VelocityTimeout(const ros::TimerEvent &event);

  robosub::AngularPosition setpoint_rpy_;
  robosub::AngularVelocity setpoint_drpy_;
  robosub::LinearPosition setpoint_xyz_;
  robosub::LinearVelocity setpoint_dxyz_;

  ros::Timer timer_;
  ros::NodeHandle nh_;
};

#endif  // MOTION_CONTROLLER_H
