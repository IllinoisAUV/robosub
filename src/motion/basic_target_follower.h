#ifndef TARGET_FOLLOWER_H
#define TARGET_FOLLOWER_H
#include <ros/ros.h>
#include <cstdint>

// Library that provides a basic target follower. Using the position of the
// target in the camera's view, motion commands are generated to point the sub
// at the target using a stupid P loop. Requires that the motion controller
// currently being used allow angular and linear velocity control
class BasicTargetFollower {
  public:
   // Args:
   //   width - width in pixels of the image
   //   height - height in pixels of the image
   BasicTargetFollower(uint32_t width, uint32_t height);

   // Method to generate motion commands based on the position on the screen.
   // Must be called around 1 Hz. Assumes that camera is centered on the sub
   // Args:
   //   x - x position of the target on the screen, based on the indexing into
   //   the image y - y position of the target on the screen, based on the
   //   indexing into the image
   void update(uint32_t x, uint32_t y);

  private:
   // Image width and height
   float x_center_, y_center_;

   ros::NodeHandle nh_;
   ros::Publisher angular_vel_pub_;
   ros::Publisher linear_vel_pub_;
};

#endif  // TARGET_FOLLOWER_H
