#include "basic_target_follower.h"

#include <ros/exception.h>
#include <string>

#include "helpers/AngularVelocity.h"
#include "helpers/LinearVelocity.h"

constexpr float kYaw = 0.01;
constexpr float kAlt = 0.01;
// Forward velocity of the sub
constexpr float kSpeed = 0.1;

BasicTargetFollower::BasicTargetFollower(uint32_t width, uint32_t height)
    : x_center_(width / 2.0), y_center_(height / 2.0) {
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

void BasicTargetFollower::update(uint32_t x, uint32_t y) {
  // Center the values around the middle of the image
  float x_err = x - x_center_;
  float y_err = y - y_center_;

  // Adjust the yaw to point at the target
  float dyaw = x_err * kYaw;
  robosub::AngularVelocity angular = AngularVelocity(0, 0, dyaw);

  // Line up altitude
  float dalt = y_err * kAlt;
  robosub::LinearVelocity linear = LinearVelocity(kSpeed, 0, dalt);

  angular_vel_pub_.publish(angular);
  linear_vel_pub_.publish(linear);
}
