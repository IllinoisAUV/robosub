#include <math.h>
#include <ros/exception.h>
#include <ros/ros.h>
#include <string>
#include "helpers/AngularVelocity.h"

// Initiates a BarrelRoll for one complete circle
class BarrelRoll {
  float droll;
  ros::Publisher angular_vel_pub_;
  ros::NodeHandle nh_;

 public:
  BarrelRoll(float droll_ = 0) {
    droll = droll_;
    std::string angular_vel_topic;
    if (!ros::param::get("angular_velocity_topic", angular_vel_topic)) {
      throw ros::Exception("Must specify angular_position_topic parameter");
    }
    angular_vel_pub_ =
        nh_.advertise<robosub::AngularVelocity>(angular_vel_topic, 1);
  }
  // Set & publish AngularVelocity with droll_
  void update() {
    robosub::AngularVelocity angular = AngularVelocity(droll, 0, 0);
    angular_vel_pub_.publish(angular);
  }
};
int main(int argc, char **argv) {
  // Value of AngularVelocity
  float droll = 0.05;
  ros::init(argc, argv, "barrelroll");
  BarrelRoll *broll = new BarrelRoll(droll);
  float count = 0;
  // Publishes velocity till one complete circle
  while (ros::ok() && count <= (int)(2 * M_PI) / droll) {
    broll->update();
    ros::spinOnce();
    count++;
  }
  return 0;
}
