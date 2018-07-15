#!/bin/bash

# Script that starts the submarine. Should be called by hand or on boot by
# systemd

set -xeu

# Maxmize the jetson's performance
sudo /home/ubuntu/jetson_clocks.sh --store
trap "sudo /home/ubuntu/jetson_clocks.sh --restore" EXIT

# Add all necessary launch files here
roslaunch robosub motion.launch &
roslaunch robosub bottom-camera.launch &

wait
