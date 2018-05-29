#include "basic_target_follower.h"
#include <ros/ros.h>
#include <string>

// Node to wrap basic_target_follower. Messages are received on the target_topic
// (specified by parameter) and published to the motion controller.
int main(int argc, char **argv) {
  ros::init(argc, argv, "targetfollower");
  ros::NodeHandle nh("~");

  std::string target_topic;
  if (!nh.getParam("target_topic", target_topic)) {
    throw ros::Exception("Must specify target_topic parameter");
  }

  if (!nh.getParam("target_topic", target_topic)) {
    throw ros::Exception("Must specify target_topic parameter");
  }

  float kSpeed = 0.0;
  if (!nh.getParam("kSpeed", kSpeed)) {
    throw ros::Exception("Must specify kSpeed parameter");
  }

  float kAlt = 0.0;
  if (!nh.getParam("kAlt", kAlt)) {
    throw ros::Exception("Must specify kAlt parameter");
  }

  float kYaw = 0.0;
  if (!nh.getParam("kYaw", kYaw)) {
    throw ros::Exception("Must specify kYaw parameter");
  }

  BasicTargetFollower tgtfollower(kSpeed, kAlt, kYaw);

  ros::Subscriber sub = nh.subscribe(
      target_topic, 10, &BasicTargetFollower::update, &tgtfollower);
  ros::spin();

  return 0;
}
