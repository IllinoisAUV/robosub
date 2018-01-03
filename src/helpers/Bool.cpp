#include "Bool.h"

std_msgs::Bool Bool(bool data) {
  std_msgs::Bool val;
  val.data = data;
  return val;
}
