#include "basic_target_follower.h"

#include <ros/exception.h>
#include <string>

#include <geometry_msgs/Twist.h>
#include "constructors/Twist.h"
#include "constructors/Vector3.h"

BasicTargetFollower::BasicTargetFollower(float kSpeed, float kAlt, float kYaw)
    : kSpeed_(kSpeed), kAlt_(kAlt), kYaw_(kYaw) {
  std::string velocity_topic = "/robosub/target_twist";
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(velocity_topic, 1);
}

void BasicTargetFollower::update(const robosub::VisualTarget::ConstPtr& msg) {
  // Center the values around the middle of the image
  float x_err = msg->x;
  float y_err = msg->y;

  // Adjust the yaw to point at the target
  float dyaw = x_err * kYaw_;
  float dalt = y_err * kAlt_;
  // geometry_msgs::TwistStamped vel = TwistStamped(
  //     std_msgs::Header(), Twist(Vector3(0, 0, dalt), Vector3(0, 0, dyaw)));
  geometry_msgs::Twist vel =
      Twist(Vector3(kSpeed_, 0, dalt), Vector3(0, 0, dyaw));

  vel_pub_.publish(vel);
}
