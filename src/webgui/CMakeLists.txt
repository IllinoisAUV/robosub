cmake_minimum_required(VERSION 2.8.3)
project(webgui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package (
  DEPENDS rospy std_msgs
)

install(PROGRAMS bin/run_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
