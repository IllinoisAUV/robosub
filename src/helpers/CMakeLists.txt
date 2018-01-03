# Library containing useful message constructors
add_library(ctors Vector3.cpp Bool.cpp AngularPosition.cpp AngularVelocity.cpp LinearPosition.cpp LinearVelocity.cpp)
add_dependencies(ctors ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
