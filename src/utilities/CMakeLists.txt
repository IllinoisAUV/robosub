add_executable(kill_switch_node kill_switch_node.cpp kill_switch.cpp gpio.cpp)
target_link_libraries(kill_switch_node ${catkin_LIBRARIES})
