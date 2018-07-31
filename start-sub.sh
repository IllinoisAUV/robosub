#!/bin/bash

# Script that starts the submarine. Should be called by hand or on boot by
# systemd

set -xu

# Remove old files if they exist
sudo rm /home/ubuntu/l4t_dfs.conf

# Maximize the jetson's performance
set -e
sudo /home/ubuntu/jetson_clocks.sh --store
trap "sudo /home/ubuntu/jetson_clocks.sh --restore" EXIT

# Add all necessary launch files here
roslaunch robosub motion.launch &
# roslaunch robosub bottom-camera.launch &

wait
