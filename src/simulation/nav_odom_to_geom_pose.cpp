#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"

ros::Publisher target_pub;

void callback(const nav_msgs::Odometry::ConstPtr& msg) {
  target_pub.publish(msg->pose.pose);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_to_pose_converter");

  ros::NodeHandle nh("~");

  std::string pose_target_topic;
  if (!nh.getParam("pose_target_topic", pose_target_topic)) {
    throw ros::Exception("Must specify target_topic parameter");
  }

  std::string odom_input_topic;
  if (!nh.getParam("odom_input_topic", odom_input_topic)) {
    throw ros::Exception("Must specify target_topic parameter");
  }

  target_pub = nh.advertise<geometry_msgs::Pose>(pose_target_topic, 1);

  ros::Subscriber sub = nh.subscribe(odom_input_topic, 1, callback);

  ros::spin();

  return 0;
}
