#!/bin/bash
set -e

# setup ros environment
source "$SYSROOT/opt/ros/kinetic/setup.bash"
exec "$@"
