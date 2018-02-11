#ifndef MAVROS_H
#define MAVROS_H

#include <ros/ros.h>
#include <ros/time.h>

#include "motion_controller.h"

// MotionController that uses mavros and RCOverride to control the sub
class MavrosRCController : public MotionController {
 public:
  MavrosRCController();

 private:
  // Timer callback from MotionController
  virtual void DoUpdate() override;
  virtual void DoArming(bool arm) override;
  ros::Publisher rc_pub_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient mode_client_;

  uint16_t angleToPpm(double angle);
  uint16_t speedToPpm(double speed);
};
#endif  // MAVROS_H
