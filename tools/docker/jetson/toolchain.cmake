SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_SYSTEM_PROCESSOR arm)

# specify the cross compiler
SET(CMAKE_C_COMPILER   $ENV{TRIPLET}-gcc)
SET(CMAKE_CXX_COMPILER $ENV{TRIPLET}-g++)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH $ENV{SYSROOT} $ENV{SYSROOT}/opt/ros/kinetic /usr/local/cuda)

# Always pass the --sysroot flag to compiler
SET(CMAKE_SYSROOT $ENV{SYSROOT})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
