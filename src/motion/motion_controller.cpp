#include "motion_controller.h"

#include <std_msgs/Header.h>

#include "constructors/Point.h"
#include "constructors/PoseStamped.h"
#include "constructors/Quaternion.h"
#include "constructors/TwistStamped.h"
#include "constructors/Vector3.h"

MotionController::MotionController() {
  // Zero all setpoints initially
  setpoint_pos_ = PoseStamped();
  setpoint_vel_ = TwistStamped();
}

void MotionController::Start() {
  node_name_ = ros::this_node::getName();

  // Start the control timer
  timer_ =
      nh_.createTimer(ros::Duration(kPeriod), &MotionController::Update, this);

  // Start subscribers
  pose_sub_ = nh_.subscribe("/cmd_pos", 1, &MotionController::SetPose, this);
  twist_sub_ = nh_.subscribe("/cmd_vel", 1, &MotionController::SetTwist, this);
  kill_sub_ = nh_.subscribe("/arming", 1, &MotionController::Arming, this);
}

void MotionController::SetPose(const geometry_msgs::PoseStamped pos) {
  ROS_INFO("%s: Received new pose setpoint", node_name_.c_str());
  setpoint_pos_ = pos;
}

void MotionController::SetTwist(const geometry_msgs::TwistStamped vel) {
  ROS_INFO("%s: Received new twist setpoint", node_name_.c_str());
  // Cancel previous timeout
  vel_timer_.stop();

  setpoint_vel_ = vel;

  // Set a timeout for 1 second
  vel_timer_ = nh_.createTimer(ros::Duration(1.0),
                               &MotionController::VelocityTimeout, this, true);
}

void MotionController::Arming(const std_msgs::Bool arm) { DoArming(arm.data); }

void MotionController::Update(const ros::TimerEvent &event) { DoUpdate(); }

// Set velocity to zero
void MotionController::VelocityTimeout(const ros::TimerEvent &event) {
  ROS_ERROR("Velocity setpoint timed out");
  setpoint_vel_ = TwistStamped(Header(), Twist());
  vel_timer_.stop();
}
