add_executable(tracking_node tracking.cpp)
add_dependencies(tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${cmt_tracking_INCLUDE_DIRS})
