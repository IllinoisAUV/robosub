#!/bin/bash

# This file builds a specified container. You probably should be looking at the
# repository root's build.sh
set -e

realpath() {
    [[ $1 = /* ]] && echo "$1" || echo "$PWD/${1#./}"
}

SCRIPT_PATH=$(realpath $0)
SCRIPT_DIR=$(dirname $SCRIPT_PATH)

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
