add_executable(rcmavros_node rcmavros_node.cpp rcmavros.cpp)
add_dependencies(rcmavros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rcmavros_node ctors ${catkin_LIBRARIES})

add_library(basic_target_follower basic_target_follower.cpp)
add_dependencies(basic_target_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
