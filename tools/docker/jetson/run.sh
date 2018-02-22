#!/bin/bash
set -e

docker run -it -u $(id -u $(whoami)) -v $(pwd):/home/ubuntu/catkin_ws -w /home/ubuntu/catkin_ws robosub:jetson catkin_make -DCMAKE_TOOLCHAIN_FILE=/toolchain.cmake -DCMAKE_BUILD_TYPE=Release $@
