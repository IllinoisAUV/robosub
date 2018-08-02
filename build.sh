#!/bin/bash
set -ex

realpath() {
    [[ $1 = /* ]] && echo "$1" || echo "$PWD/${1#./}"
}

SCRIPT_PATH=$(realpath $0)
ROOT_DIR=$(dirname $SCRIPT_PATH)


function help() {
    echo "Usage: $(basename "$0") [options] [target] [args]"
    echo "Options:"
    echo "  -h, --help                      Print this help message"
    echo ""
    echo "Targets:"
    echo "  host (Default)                  Builds code to be run on host into build/host/"
    echo "  download                        Downloads the code to the jetson"
    echo "  clean                           Cleans build folders in build/"
    echo "  clang-format                    Runs clang format on all C++ code"
    echo ""
    echo "Args:"
    echo "  --make-args=clang-format-check  Checks if all C++ is clang-format compliant"
    echo "  --make-args=pylint              Runs pylint on all python code"
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    help
    exit 0
fi


DOWNLOAD=
TARGET=host
if [ "$1" == "download" ]; then
    ssh ubuntu@10.0.0.2 chmod -R u+w /home/ubuntu/catkin_ws/src/robosub
    rsync -r --delete --update $ROOT_DIR ubuntu@10.0.0.2:~/catkin_ws/src/robosub/
    ssh ubuntu@10.0.0.2 chmod -R aug-w /home/ubuntu/catkin_ws/src/robosub
    exit 0
fi

case $TARGET in
    host);;
    clang-format)
        docker run -it -v $(pwd):/catkin_ws/src/robosub -w /catkin_ws/ \
            illinoisauv/robosub:latest catkin_make --make-args clang-format
        exit 0;
        ;;
    clean)
        rm -rf build/*
        exit 0
        ;;
    *)
        help
        exit 1
        ;;
esac

# Build necessary docker container
echo "Building required docker container"
# $ROOT_DIR/tools/docker/build.sh $TARGET

# Make catkin_ws to build into
mkdir -p build/$TARGET/src/$(basename $ROOT_DIR)
rsync -r --delete --update --exclude=.git --exclude=build . build/$TARGET/src/$(basename $ROOT_DIR)

# Do the build
pushd build/$TARGET
$ROOT_DIR/tools/docker/$TARGET/run.sh ${@:2}
popd


