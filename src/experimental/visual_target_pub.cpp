#include <stdlib.h> /* srand, rand */
#include <sstream>
#include "robosub/VisualTarget.h"
#include "ros/ros.h"

/* This file is to test if the basic_target_follower works properly.*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher target_pub =
      n.advertise<robosub::VisualTarget>("/rexrov/target", 10);

  ros::Rate loop_rate(10);

  int rand_target;
  while (ros::ok()) {
    robosub::VisualTarget msg;
    rand_target = rand() % 100 + 1;

    // since our images are 480p
    msg.x = 320 + rand_target;
    msg.y = 240 - rand_target;

    target_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
