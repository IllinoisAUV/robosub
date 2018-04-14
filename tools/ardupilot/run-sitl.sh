#!/bin/bash

set -xe

realpath() {
    [[ $1 = /* ]] && echo "$1" || echo "$PWD/${1#./}"
}

SCRIPT_PATH=$(realpath $0)
ROOT_DIR=$(dirname $SCRIPT_PATH)

docker build -t robosub:ardusub $ROOT_DIR
sudo docker run --net=host -v $ROOT_DIR/mav.parm:/ardupilot/ArduSub/mav.parm -it robosub:ardusub
