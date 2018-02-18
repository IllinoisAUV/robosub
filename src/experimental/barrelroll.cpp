#include <ros/exception.h>
#include <ros/ros.h>
#include <cmath>
#include <string>
#include "helpers/AngularVelocity.h"

class BarrelRoll {
  float droll;
  ros::Publisher angular_vel_pub_;
  ros::NodeHandle nh_;

 public:
  BarrelRoll(float droll_) {
    droll = droll_;
    std::string angular_vel_topic;
    if (!ros::param::get("angular_velocity_topic", angular_vel_topic)) {
      throw ros::Exception("Must specify angular_position_topic parameter");
    }
    angular_vel_pub_ =
        nh_.advertise<robosub::AngularVelocity>(angular_vel_topic, 1);
  }
  void update() {
    robosub::AngularVelocity angular = AngularVelocity(droll, 0, 0);

    angular_vel_pub_.publish(angular);
  }

  void zeroVelocity() {
    robosub::AngularVelocity angul = AngularVelocity(0, 0, 0);
    angular_vel_pub_.publish(angul);
  }
};
int main() {
  float droll = 0.05;
  BarrelRoll *broll = new BarrelRoll(droll);
  ros::Rate loop_rate(10);
  float count = 0;
  while (ros::ok() && count <= (2 * M_PI) * 10 / droll) {
    broll->update();
    ros::spinOnce();
    count++;
    loop_rate.sleep();
  }
  broll->zeroVelocity();
  return 0;
}
