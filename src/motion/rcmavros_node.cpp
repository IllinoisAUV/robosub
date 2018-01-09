#include <fcntl.h>
#include <ros/exception.h>
#include <ros/ros.h>
#include <signal.h>
#include <string>

#include "helpers/AngularPosition.h"
#include "helpers/AngularVelocity.h"
#include "helpers/Bool.h"
#include "helpers/LinearPosition.h"
#include "helpers/LinearVelocity.h"
#include "helpers/Vector3.h"
#include "helpers/param.h"

#include "motion_controller.h"
#include "rcmavros.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void sigIntHandler(int sig) { g_request_shutdown = 1; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_controller");
  ros::NodeHandle nh;

  MotionController *controller = new MavrosRCController();

  // This must come after the nodehandle is initialized because any new
  // NodeHandles will reregister the default signal handlers
  signal(SIGINT, sigIntHandler);

  // Get all of the necessary parameters
  std::string rpy_topic;
  std::string drpy_topic;
  std::string xyz_topic;
  std::string dxyz_topic;
  std::string arming_topic;
  getParamOrThrow("/angular_position_topic", rpy_topic);
  getParamOrThrow("/angular_velocity_topic", drpy_topic);
  getParamOrThrow("/linear_position_topic", xyz_topic);
  getParamOrThrow("/linear_velocity_topic", dxyz_topic);
  getParamOrThrow("/arming_topic", arming_topic);

  ros::Subscriber rpy_sub =
      nh.subscribe(rpy_topic, 1, &MotionController::SetRPY, controller);
  ros::Subscriber drpy_sub =
      nh.subscribe(drpy_topic, 1, &MotionController::SetdRPY, controller);
  ros::Subscriber xyz_sub =
      nh.subscribe(xyz_topic, 1, &MotionController::SetXYZ, controller);
  ros::Subscriber dxyz_sub =
      nh.subscribe(dxyz_topic, 1, &MotionController::SetdXYZ, controller);
  ros::Subscriber kill_sub =
      nh.subscribe(arming_topic, 10, &MotionController::arming, controller);

  // Turn off any movement
  controller->SetdRPY(AngularVelocity(0.0, 0.0, 0.0));
  controller->SetdXYZ(LinearVelocity(0.0, 0.0, 0.0));

  while (!g_request_shutdown) {
    ros::spinOnce();
  }

  controller->SetdRPY(AngularVelocity(0.0, 0.0, 0.0));
  controller->SetdXYZ(LinearVelocity(0.0, 0.0, 0.0));
  controller->arming(Bool(false));
}
