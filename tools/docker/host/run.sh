#!/bin/bash
set -e

docker run -it -u $(id -u $(whoami)) -v $(pwd):$(pwd) -w $(pwd) robosub:host catkin_make $@
