#include <ros/ros.h>

#include "helpers/AngularPosition.h"
#include "helpers/AngularVelocity.h"
#include "helpers/Bool.h"
#include "helpers/LinearPosition.h"
#include "helpers/LinearVelocity.h"
#include "helpers/Vector3.h"

#include "motion_controller.h"
#include "rcmavros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_controller");

  // Construction should disable movement
  MotionController *controller = new MavrosRCController();

  controller->Start();

  // Run until ctrl-c is pressed
  ros::spin();
}
