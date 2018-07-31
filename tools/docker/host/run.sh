#!/bin/bash
set -e

docker run -it -u $(id -u $(whoami)) -v $(pwd):$(pwd) -w $(pwd) \
    illinoisauv/robosub:latest catkin_make $@
