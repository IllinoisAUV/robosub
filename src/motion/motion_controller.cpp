#include "motion_controller.h"

#include "helpers/AngularPosition.h"
#include "helpers/AngularVelocity.h"
#include "helpers/LinearPosition.h"
#include "helpers/LinearVelocity.h"

MotionController::MotionController() {
  // Zero all setpoints initially
  setpoint_rpy_ = AngularPosition(0, 0, 0);
  setpoint_drpy_ = AngularVelocity(0, 0, 0);
  setpoint_xyz_ = LinearPosition(0, 0, 0);
  setpoint_dxyz_ = LinearVelocity(0, 0, 0);
}

void MotionController::Start() {
  // Start the control timer
  timer_ =
      nh_.createTimer(ros::Duration(kPeriod), &MotionController::Update, this);

  // Start subscribers
  ros::Subscriber rpy_sub = nh_.subscribe("/setpoint/angular/position", 1,
                                          &MotionController::SetRPY, this);
  ros::Subscriber drpy_sub = nh_.subscribe("/setpoint/angular/velocity", 1,
                                           &MotionController::SetdRPY, this);
  ros::Subscriber xyz_sub = nh_.subscribe("/setpoint/linear/position", 1,
                                          &MotionController::SetXYZ, this);
  ros::Subscriber dxyz_sub = nh_.subscribe("/setpoint/linear/velocity", 1,
                                           &MotionController::SetdXYZ, this);
  ros::Subscriber kill_sub =
      nh_.subscribe("/arming", 1, &MotionController::Arming, this);
}

void MotionController::SetRPY(const robosub::AngularPosition rpy) {
  setpoint_rpy_ = rpy;
}

void MotionController::SetdRPY(const robosub::AngularVelocity drpy) {
  static ros::Timer timer;
  // Cancel previous timeout
  timer.stop();

  setpoint_drpy_ = drpy;

  // Set a timeout for 1 second if the velocity is not zero
  if (drpy.droll != 0.0 || drpy.dpitch != 0.0 || drpy.dyaw != 0.0) {
    // Set a timeout for 1 second
    timer = nh_.createTimer(ros::Duration(1.0),
                            &MotionController::VelocityTimeout, this, true);
  }
}

void MotionController::SetXYZ(const robosub::LinearPosition xyz) {
  setpoint_xyz_ = xyz;
}

void MotionController::SetdXYZ(const robosub::LinearVelocity dxyz) {
  static ros::Timer timer;
  // Cancel previous timeout
  timer.stop();

  setpoint_dxyz_ = dxyz;

  // Set a timeout for 1 second if the velocity is not zero
  if (dxyz.dx != 0.0 || dxyz.dy != 0.0 || dxyz.dz != 0.0) {
    timer = nh_.createTimer(ros::Duration(1.0),
                            &MotionController::VelocityTimeout, this, true);
  }
}

void MotionController::Arming(const std_msgs::Bool arm) { DoArming(arm.data); }

void MotionController::Update(const ros::TimerEvent &event) { DoUpdate(); }

// Set velocity to zero
void MotionController::VelocityTimeout(const ros::TimerEvent &event) {
  setpoint_drpy_ = AngularVelocity(0.0, 0.0, 0.0);
  setpoint_dxyz_ = LinearVelocity(0.0, 0.0, 0.0);
}
