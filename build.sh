#!/bin/bash
set -ex

SCRIPT_DIR="$(readlink -f $(dirname $0))"

docker run -v $SCRIPT_DIR:/catkin_ws -it catkin_builder
