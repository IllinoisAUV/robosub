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
  // Start the control timer
  timer_ =
      nh_.createTimer(ros::Duration(kPeriod), &MotionController::Update, this);

  // Start subscribers
  ros::Subscriber pose_sub =
      nh_.subscribe("/cmd_pos", 1, &MotionController::SetPos, this);
  ros::Subscriber twist_sub =
      nh_.subscribe("/cmd_vel", 1, &MotionController::SetVel, this);
  ros::Subscriber kill_sub =
      nh_.subscribe("/arming", 1, &MotionController::Arming, this);
}

void MotionController::SetPos(const geometry_msgs::PoseStamped pos) {
  setpoint_pos_ = pos;
}

void MotionController::SetVel(const geometry_msgs::TwistStamped vel) {
  static ros::Timer timer;
  // Cancel previous timeout
  timer.stop();

  setpoint_vel_ = vel;

  // Set a timeout for 1 second
  timer = nh_.createTimer(ros::Duration(1.0),
                          &MotionController::VelocityTimeout, this, true);
}

void MotionController::Arming(const std_msgs::Bool arm) { DoArming(arm.data); }

void MotionController::Update(const ros::TimerEvent &event) { DoUpdate(); }

// Set velocity to zero
void MotionController::VelocityTimeout(const ros::TimerEvent &event) {
  setpoint_vel_ = TwistStamped(Header(), Twist());
}
