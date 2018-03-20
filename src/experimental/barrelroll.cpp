#include <ros/exception.h>
#include <ros/ros.h>
#include <cmath>
#include <string>

#include "helpers/TwistStamped.h"

class BarrelRoll {
  float droll;
  ros::Publisher vel_pub_;
  ros::NodeHandle nh_;

 public:
  BarrelRoll(float droll_) {
    droll = droll_;
    std::string vel_topic;
    if (!ros::param::get("velocity_topic", vel_topic)) {
      throw ros::Exception("Must specify velocity_topic parameter");
    }
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(vel_topic, 1);
  }
  void update() {
    geometry_msgs::TwistStamped msg = TwistStamped(
        std_msgs::Header(), Twist(Vector3(), Vector3(droll, 0, 0)));

    vel_pub_.publish(msg);
  }

  void zeroVelocity() {
    // Send a zero message
    geometry_msgs::TwistStamped msg = TwistStamped();
    vel_pub_.publish(msg);
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
