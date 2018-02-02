#include "rcmavros.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <string>

using robosub::AngularPosition;
using robosub::AngularVelocity;
using robosub::LinearPosition;
using robosub::LinearVelocity;
using std_msgs::Bool;

using mavros_msgs::CommandBool;
using mavros_msgs::OverrideRCIn;
using mavros_msgs::SetMode;

const std::string kModeDepthHold = "ALT_HOLD";
const std::string kModeManual = "MANUAL";
const std::string kModeStabilize = "STABILIZE";

constexpr float kPeriod = 0.1;

MavrosRCController::MavrosRCController() {
  rc_pub_ = nh_.advertise<OverrideRCIn>("/mavros/rc/override", 10);

  // Service clients for arming and changing the mode of the sub
  arming_client_ = nh_.serviceClient<CommandBool>("/mavros/cmd/arming");
  mode_client_ = nh_.serviceClient<SetMode>("mavros/set_mode");

  SetMode mode_cmd;
  mode_cmd.request.base_mode = 0;
  mode_cmd.request.custom_mode = kModeDepthHold;

  if (mode_client_.call(mode_cmd)) {
    ROS_INFO("Mode changed");
  } else {
    ROS_ERROR("Failed to set mavros mode");
    // TODO: Handle error
  }

  // Zero all of the setpoints
  setpoint_rpy_.roll = setpoint_rpy_.pitch = setpoint_rpy_.yaw = 0.0;
  setpoint_drpy_.droll = setpoint_drpy_.dpitch = setpoint_drpy_.dyaw = 0.0;
  setpoint_xyz_.x = setpoint_xyz_.y = setpoint_xyz_.z = 0.0;
  setpoint_dxyz_.dx = setpoint_dxyz_.dy = setpoint_dxyz_.dz = 0.0;

  // Start the control timer
  timer_ =
      nh_.createTimer(ros::Duration(kPeriod), &MavrosRCController::callback, this);
}

void MavrosRCController::SetRPY(const AngularPosition rpy) {
  // Is actually a no-op due to lack of position control
  setpoint_rpy_ = rpy;
}

void MavrosRCController::SetdRPY(const AngularVelocity drpy) {
  setpoint_drpy_ = drpy;
}

void MavrosRCController::SetXYZ(const LinearPosition xyz) {
  // Is actually a no-op due to lack of position control
  setpoint_xyz_ = xyz;
}

void MavrosRCController::SetdXYZ(const LinearVelocity dxyz) {
  setpoint_dxyz_ = dxyz;
}

// Send a message to mavros to arm or disarm the sub
void MavrosRCController::arming(const Bool arm) {
  mavros_msgs::CommandBool srv;
  srv.request.value = arm.data;
  if (!arming_client_.call(srv)) {
    ROS_INFO("Failed to disarm");
    return;
  }
}

// Converts angles in radians to RC signal
uint16_t MavrosRCController::angleToPpm(double angle) {
  // Map [-pi, pi] -> [1000, 2000]
  uint16_t ppm = (angle - (-M_PI)) / (M_PI - (-M_PI)) * (1000) + 1000;
  return ppm;
}

// Convert a linear speed (0.0-1.0) to RC signal
uint16_t MavrosRCController::speedToPpm(double speed) {
  if (speed > 1.0 || speed < -1.0) {
    ROS_ERROR("Invalid speed requested: %f", speed);
    return 1500;
  }
  return 1500 + speed * 500.0;
}

// Control callback
void MavrosRCController::callback(const ros::TimerEvent &e) {
  OverrideRCIn msg;

  // Account for velocity setpoints
  setpoint_rpy_.roll += setpoint_drpy_.droll * kPeriod;
  setpoint_rpy_.pitch += setpoint_drpy_.dpitch * kPeriod;

  msg.channels[1] = angleToPpm(setpoint_rpy_.roll);
  msg.channels[0] = angleToPpm(setpoint_rpy_.pitch);
  msg.channels[3] = speedToPpm(setpoint_rpy_.dyaw);

  msg.channels[5] = speedToPpm(setpoint_dxyz_.dx);
  msg.channels[6] = speedToPpm(setpoint_dxyz_.dy);
  msg.channels[2] = speedToPpm(setpoint_dxyz_.dz);

  msg.channels[4] = 1500;
  rc_pub_.publish(msg);
}
