#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

class MotionController {
 public:
  MotionController();
  virtual ~MotionController() {}

  void Start();

  // Set pose target
  void SetPos(const geometry_msgs::PoseStamped pose);

  // Set velocity target
  void SetVel(const geometry_msgs::TwistStamped vel);

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

  geometry_msgs::PoseStamped setpoint_pos_;
  geometry_msgs::TwistStamped setpoint_vel_;

  ros::Timer timer_;
  ros::NodeHandle nh_;
};

#endif  // MOTION_CONTROLLER_H
