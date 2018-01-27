# Add the current directory to the list of python modules
list(APPEND PYTHON_MODULES ${CMAKE_CURRENT_SOURCE_DIR})

catkin_install_python(PROGRAMS mission_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Node for tests
catkin_install_python(PROGRAMS test/test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/missions.test)
endif()
