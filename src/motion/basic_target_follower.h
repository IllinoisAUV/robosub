#ifndef TARGET_FOLLOWER_H
#define TARGET_FOLLOWER_H
#include <ros/ros.h>
#include <cstdint>

#include "robosub/VisualTarget.h"

// Library that provides a basic target follower. Using the position of the
// target in the camera's view, motion commands are generated to point the sub
// at the target using a stupid P loop. Requires that the motion controller
// currently being used allow angular and linear velocity control
class BasicTargetFollower {
 public:
  BasicTargetFollower(float kSpeed, float kAlt, float kYaw);

  // Method to generate motion commands based on the position on the screen.
  // Must be called around 1 Hz. Assumes that camera is centered on the sub
  // Args:
  //  msg->x position of the target on the screen, relative to the center.
  //  msg->y position of the target on the screen, relative to the center.
  void update(const robosub::VisualTarget::ConstPtr& msg);

  void setkAlt(float kAlt) { kAlt_ = kAlt; }

  void setkYaw(float kYaw) { kYaw_ = kYaw; }
  void setkSpeed(float kSpeed) { kSpeed_ = kSpeed; }

 private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;

  float kSpeed_;
  float kAlt_;
  float kYaw_;
};

#endif  // TARGET_FOLLOWER_H
