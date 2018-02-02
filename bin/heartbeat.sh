#!/bin/bash

while true; do
    echo "Testing connection"
    ping -W 1 -c1 10.0.0.1 2>&1 > /dev/null && echo "Connected!" || rosservice call /mavros/cmd/arming 0 
    sleep 0.5
done
