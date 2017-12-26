#ifndef KILL_SWITCH_H
#define KILL_SWITCH_H
#include "ros/ros.h"

#include "gpio.h"

class KillSwitch {
public:
  KillSwitch(unsigned int pin, std::string topic);
  ~KillSwitch();
  void Run();
private:
  void publishMessage(bool value);
  ros::NodeHandle node_;
  GPIO pin_;
  ros::Publisher pub_;
};

#endif // KILL_SWITCH_H
