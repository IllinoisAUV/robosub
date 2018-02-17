#!/bin/bash

# This file builds a specified container. You probably should be looking at the
# repository root's build.sh
set -e

SCRIPT_DIR=$(dirname $(readlink -f $0))

case $1 in 
    host)
        docker build -t robosub:host $SCRIPT_DIR/host
        ;;
    jetson)
        docker build -t robosub:jetson $SCRIPT_DIR/jetson
        ;;
    all)
        docker build -t robosub:host $SCRIPT_DIR/host
        docker build -t robosub:jetson $SCRIPT_DIR/jetson
        ;;
    *)
        echo "Please specify a target in [host,jetson]"
        exit 1
esac
