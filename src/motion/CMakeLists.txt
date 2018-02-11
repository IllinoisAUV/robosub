add_executable(motion_controller_node motion_controller.cpp rcmavros.cpp motion_controller_node.cpp)
add_dependencies(motion_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_controller_node ctors ${catkin_LIBRARIES})

add_library(basic_target_follower basic_target_follower.cpp)
add_dependencies(basic_target_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
