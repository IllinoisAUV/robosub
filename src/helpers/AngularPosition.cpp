#include "AngularPosition.h"

robosub::AngularPosition AngularPosition(double roll, double pitch,
                                         double yaw) {
  robosub::AngularPosition val;
  val.roll = roll;
  val.pitch = pitch;
  val.yaw = yaw;
  return val;
}
