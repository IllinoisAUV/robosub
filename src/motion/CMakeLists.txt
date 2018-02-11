library(motion_controller motion_controller.cpp)
library(rcmavros rcmavros.cpp)

library(basic_target_follower basic_target_follower.cpp)

node(motion_controller_node motion_controller_node.cpp motion_controller rcmavros ctors)
