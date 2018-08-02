#include "rcmavros.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <tf/tf.h>
#include <string>

#include <unistd.h>

#include "constructors/Quaternion.h"
#include "constructors/Vector3.h"

using std_msgs::Bool;

using mavros_msgs::CommandBool;
using mavros_msgs::OverrideRCIn;
using mavros_msgs::SetMode;

const std::string kModeDepthHold = "ALT_HOLD";
const std::string kModeManual = "MANUAL";
const std::string kModeStabilize = "STABILIZE";

const std::string kModeService = "/mavros/set_mode";
const std::string kArmingService = "/mavros/cmd/arming";

MavrosRCController::MavrosRCController() {
  usleep(2 * 1000000);  // 2 sec sleep

  // Wait for the mavros node to come up
  if (!ros::service::waitForService(kModeService)) {
    throw ros::Exception(
        "Failed to wait for /mavros/set_mode to become available");
  }

  if (!ros::service::waitForService(kArmingService)) {
    throw ros::Exception(
        "Failed to wait for /mavros/cmd/arming to become available");
  }

  rc_pub_ = nh_.advertise<OverrideRCIn>("/mavros/rc/override", 1);

  // Service clients for arming and changing the mode of the sub
  arming_client_ = nh_.serviceClient<CommandBool>(kArmingService);
  mode_client_ = nh_.serviceClient<SetMode>(kModeService);

  SetMode mode_cmd;
  mode_cmd.request.base_mode = 0;
  mode_cmd.request.custom_mode = kModeDepthHold;

  if (!mode_client_.call(mode_cmd)) {
    ROS_ERROR("Failed to set mavros mode");
  }
}

// Send a message to mavros to arm or disarm the sub
void MavrosRCController::DoArming(bool arm) {
  ROS_INFO(arm ? "Arming" : "Disarming");
  mavros_msgs::CommandBool srv;
  srv.request.value = arm;
  if (!arming_client_.call(srv)) {
    ROS_ERROR("Failed to %s", (arm ? "arm" : "disarm"));
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
void MavrosRCController::DoUpdate() {
  OverrideRCIn msg;

  // Turn velocity setpoints into pose setpoints
  // tf::Quaternion quat(
  //     setpoint_pos_.pose.orientation.x, setpoint_pos_.pose.orientation.y,
  //     setpoint_pos_.pose.orientation.z, setpoint_pos_.pose.orientation.w);
  // tf::Matrix3x3 mat(quat);
  double roll = 0, pitch = 0, yaw = 0;
  // mat.getRPY(roll, pitch, yaw);

  roll += roll + setpoint_vel_.twist.angular.x * kPeriod;
  pitch += pitch + setpoint_vel_.twist.angular.y * kPeriod;

  setpoint_pos_.pose.orientation = QuaternionRPY(roll, pitch, yaw);

  // Send target message to ArduPilot
  msg.channels[1] = angleToPpm(roll);
  msg.channels[0] = angleToPpm(pitch);
  msg.channels[3] = speedToPpm(setpoint_vel_.twist.angular.z);

  msg.channels[5] = speedToPpm(setpoint_vel_.twist.linear.x);
  msg.channels[6] = speedToPpm(setpoint_vel_.twist.linear.y);
  msg.channels[2] = speedToPpm(setpoint_vel_.twist.linear.z);

  msg.channels[4] = 1500;
  rc_pub_.publish(msg);
}
