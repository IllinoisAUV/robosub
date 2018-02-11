#!/bin/bash
set -e

docker run -it -u $(id -u $(whoami)) -v $(pwd):/catkin_ws robosub:host catkin_make $@
