#include "Vector3.h"

geometry_msgs::Vector3 Bool(double x, double y, double z) {
  geometry_msgs::Vector3 val;
  val.x = x;
  val.y = y;
  val.z = z;
  return val;
}
