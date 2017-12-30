#include <stddef.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <string>

#include "gpio.h"

using std::string;

int main(int argc, char **argv) {
  ros::init(argc, argv, "switch");

  ros::NodeHandle nh;

  string topic;
  int num;
  bool active_high;
  if(!ros::param::get("~topic", topic)) {
    ROS_ERROR("Please specifiy a topic");
    return 1;
  }
  if(!ros::param::get("~pin", num)) {
    ROS_ERROR("Please specify a pin");
    return 1;
  }
  if(!ros::param::get("~active_high", active_high)) {
    ROS_ERROR("Please specify a logic level");
    return 1;
  }
  ROS_INFO("Starting switch node (%s) on pin %d publishing to %s", 
      topic.c_str(), num, active_high ? "active high" : "active low");


  GPIO pin(num);
  pin.SetDirection(GPIO::IN);
  pin.SetActiveState(active_high ? GPIO::ACTIVE_HIGH : GPIO::ACTIVE_LOW);

  ros::Publisher pub = nh.advertise<std_msgs::Bool>(topic, 1);

  while(pub.getNumSubscribers() == 0);

  // Get initial state
  GPIO::LogicLevel value = pin.GetValue();
  bool pressed = (value == GPIO::HIGH);
  bool last_pressed = pressed;

  // Publish initial state
  std_msgs::Bool msg;
  msg.data = last_pressed;
  ROS_DEBUG("Switch %s on pin %d: %s", topic.c_str(), num, msg.data ? "pressed" : "released");
  pub.publish(msg);

  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
    value = pin.GetValue();
    pressed = (value == GPIO::HIGH);
    if (pressed != last_pressed) {
      // Transition
      msg.data = pressed;
      ROS_DEBUG("Switch %s on pin %d: %s", topic.c_str(), num, msg.data ? "pressed" : "released");
      pub.publish(msg);
    }
    last_pressed = pressed;
    ros::spinOnce();
  }
}
