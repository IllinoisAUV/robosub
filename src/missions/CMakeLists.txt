catkin_install_python(PROGRAMS mission_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Node for tests
catkin_install_python(PROGRAMS test/test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/missions.test)
endif()
