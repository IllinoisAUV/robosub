add_executable(barrelroll barrelroll.cpp)
add_dependencies(barrelroll ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(barrelroll ${catkin_LIBRARIES})
