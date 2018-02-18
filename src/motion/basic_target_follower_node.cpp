#include "basic_target_follower.h"
#include <ros/ros.h>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "targetfollower");
  std::string target_topic;
  if (!ros::param::get("target_topic", target_topic)) {
    throw ros::Exception("Must specify target_topic parameter");
  }
  ros::NodeHandle n;
  BasicTargetFollower tgtfollower;
  ros::Subscriber sub =
      n.subscribe(target_topic, 10, &BasicTargetFollower::update, &tgtfollower);
  ros::spin();

  return 0;
}
