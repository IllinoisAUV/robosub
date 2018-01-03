#include "LinearPosition.h"

robosub::LinearPosition LinearPosition(double x, double y, double z) {
  robosub::LinearPosition val;
  val.x = x;
  val.y = y;
  val.z = z;
  return val;
}
