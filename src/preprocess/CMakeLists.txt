add_executable(preprocess_node preprocess_node.cpp filters/bilateral_filter.cpp filters/whitebalance.cpp filters/median_subtract.cpp)
add_dependencies(preprocess_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(preprocess_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(preprocess_test preprocess_test.cpp filters/bilateral_filter.cpp filters/whitebalance.cpp filters/median_subtract.cpp)
add_dependencies(preprocess_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(preprocess_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
