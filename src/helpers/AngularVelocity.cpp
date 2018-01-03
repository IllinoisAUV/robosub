#include "AngularVelocity.h"

robosub::AngularVelocity AngularVelocity(double droll, double dpitch,
                                         double dyaw) {
  robosub::AngularVelocity val;
  val.droll = droll;
  val.dpitch = dpitch;
  val.dyaw = dyaw;
  return val;
}
