#include "basic_target_follower.h"

#include <ros/exception.h>
#include <string>

#include "helpers/AngularVelocity.h"
#include "helpers/LinearVelocity.h"

BasicTargetFollower::BasicTargetFollower(float kSpeed, float kAlt, float kYaw)
    : kSpeed_(kSpeed), kAlt_(kAlt), kYaw_(kYaw) {
  std::string angular_vel_topic;
  std::string linear_vel_topic;
  if (!ros::param::get("angular_velocity_topic", angular_vel_topic)) {
    throw ros::Exception("Must specify angular_position_topic parameter");
  }
  if (!ros::param::get("linear_velocity_topic", linear_vel_topic)) {
    throw ros::Exception("Must specify angular_position_topic parameter");
  }

  angular_vel_pub_ =
      nh_.advertise<robosub::AngularVelocity>(angular_vel_topic, 1);
  linear_vel_pub_ = nh_.advertise<robosub::LinearVelocity>(linear_vel_topic, 1);
}

void BasicTargetFollower::update(const robosub::VisualTarget::ConstPtr& msg) {
  // Center the values around the middle of the image
  float x_err = msg->x;
  float y_err = msg->y;

  // Adjust the yaw to point at the target
  float dyaw = x_err * kYaw_;
  robosub::AngularVelocity angular = AngularVelocity(0, 0, dyaw);

  // Line up altitude
  float dalt = y_err * kAlt_;
  robosub::LinearVelocity linear = LinearVelocity(kSpeed_, 0, dalt);

  angular_vel_pub_.publish(angular);
  linear_vel_pub_.publish(linear);
}
