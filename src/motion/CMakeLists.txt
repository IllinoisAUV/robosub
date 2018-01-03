add_executable(motion_controller_node motion_controller_node.cpp rcmavros.cpp)
add_dependencies(motion_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_controller_node ctors ${catkin_LIBRARIES})
