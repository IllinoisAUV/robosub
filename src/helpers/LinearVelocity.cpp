#include "LinearVelocity.h"

robosub::LinearVelocity LinearVelocity(double dx, double dy, double dz) {
  robosub::LinearVelocity val;
  val.dx = dx;
  val.dy = dy;
  val.dz = dz;
  return val;
}
