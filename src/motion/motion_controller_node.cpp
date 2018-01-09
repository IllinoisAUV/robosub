#include <fcntl.h>
#include <ros/ros.h>
#include <signal.h>

#include "helpers/AngularPosition.h"
#include "helpers/AngularVelocity.h"
#include "helpers/Bool.h"
#include "helpers/LinearPosition.h"
#include "helpers/LinearVelocity.h"
#include "helpers/Vector3.h"

#include "motion_controller.h"
#include "rcmavros.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void sigIntHandler(int sig) { g_request_shutdown = 1; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_controller");

  // This must come after the nodehandle is initialized because any new
  // NodeHandles will reregister the default signal handlers
  signal(SIGINT, sigIntHandler);

  MotionController *controller = new MavrosRCController();

  ros::NodeHandle nh;
  ros::Subscriber rpy_sub = nh.subscribe("/setpoint/angular/position", 1,
                                         &MotionController::SetRPY, controller);
  ros::Subscriber drpy_sub = nh.subscribe(
      "/setpoint/angular/velocity", 1, &MotionController::SetdRPY, controller);
  ros::Subscriber xyz_sub = nh.subscribe("/setpoint/linear/position", 1,
                                         &MotionController::SetXYZ, controller);
  ros::Subscriber dxyz_sub = nh.subscribe(
      "/setpoint/linear/velocity", 1, &MotionController::SetdXYZ, controller);
  ros::Subscriber kill_sub =
      nh.subscribe("/arming", 10, &MotionController::arming, controller);

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
