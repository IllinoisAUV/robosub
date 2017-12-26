#include "kill_switch.h"

#include <stddef.h>
#include <string>
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Bool.h"

#include "gpio.h"


const bool kReverse = true;

KillSwitch::KillSwitch(unsigned int pin, std::string topic) : node_(), pin_(pin) {
  pin_.SetDirection(GPIO::IN);
  pin_.SetActiveState(GPIO::ACTIVE_HIGH);
  pub_ = node_.advertise<std_msgs::Bool>(topic, 1);
}

KillSwitch::~KillSwitch() {}

void KillSwitch::publishMessage(bool value) {
  // Build the message
  std_msgs::Bool msg;
  msg.data = value;

  ROS_INFO("Kill Switch: %s", msg.data ? "pressed" : "released");
  pub_.publish(msg);
}

void KillSwitch::Run() {

  while(pub_.getNumSubscribers() == 0);

  GPIO::LogicLevel value = pin_.GetValue();

  bool pressed = (value == GPIO::HIGH);
  bool last_pressed = pressed;
  publishMessage(pressed);

  ros::Rate rate(10);
  while(ros::ok()) {
      rate.sleep();
      value = pin_.GetValue();
      pressed = (value == GPIO::HIGH);
      if (pressed != last_pressed) {
          // Transition
          publishMessage(pressed);
      }
      last_pressed = pressed;
      ros::spinOnce();
  }
}
