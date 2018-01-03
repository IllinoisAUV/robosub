#ifndef MAVROS_H
#define MAVROS_H

#include <ros/ros.h>
#include <ros/time.h>

#include "motion_controller.h"

// MotionController that uses mavros and RCOverride to control the sub
class MavrosRCController : public MotionController {
  public:
    MavrosRCController();
    // Set roll, pitch and yaw
    virtual void SetRPY(const robosub::AngularPosition rpy);

    // Set roll, pitch and yaw velocity
    virtual void SetdRPY(const robosub::AngularVelocity drpy);

    // Set XYZ position
    virtual void SetXYZ(const robosub::LinearPosition xyz);

    // Set XYZ velocity in body frame
    virtual void SetdXYZ(const robosub::LinearVelocity dxyz);

    virtual void arming(const std_msgs::Bool arm);

  private:
    ros::NodeHandle nh_;
    ros::Publisher rc_pub_;
    ros::Timer timer_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient mode_client_;

    uint16_t angleToPpm(double angle);
    uint16_t speedToPpm(double speed);

    void callback(const ros::TimerEvent &e);

    robosub::AngularPosition setpoint_rpy_;
    robosub::AngularVelocity setpoint_drpy_;
    robosub::LinearPosition setpoint_xyz_;
    robosub::LinearVelocity setpoint_dxyz_;
};
#endif // MAVROS_H
