#include "ros/ros.h"
#include "ros/console.h"
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "kill_switch.h"


const int pin = 186;


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "kill_switch", ros::init_options::NoSigintHandler);

  ROS_INFO("Starting kill switch node");

  ros::NodeHandle nh;

  int pin;
  if(!nh.getParam("pin", pin)) {
    std::cerr << "Please specify pin parameter" << std::endl;
    exit(1);
  }

  std::string topic;
  if(!nh.getParam("topic", topic)) {
    std::cerr << "Please specify topic parameter" << std::endl;
    exit(1);
  }

  KillSwitch ks(pin, topic);
  ks.Run();
}
