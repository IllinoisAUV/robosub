#!/bin/bash
set -e
ROOT_DIR=$(dirname $(readlink -f $0))


function help() {
    echo "Usage: $(basename "$0") [options] [target] [args]"
    echo "Options:"
    echo "  -h                          Print this help message"
    echo ""
    echo "Targets:"
    echo "  host                        Builds code to be run on host into build/host/"
    echo "  jetson                      Builds code to be run on jetson into build/jetson/"
    echo "  clean                       Cleans build folders in build/"
    echo ""
    echo "Args:"
    echo "  --make-args=clangformat     Formats all c++ using clang-format"
    echo "  --make-args=pylint          Runs pylint on all python code"
}

if [ "$1" == "-h" ]; then
    help
    exit 0
fi

TARGET=$1
case $TARGET in
    host);;
    jetson);;
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
$ROOT_DIR/tools/docker/build.sh $TARGET

# Make catkin_ws to build into
mkdir -p build/$TARGET/src/$(basename $ROOT_DIR)
rsync -r --delete --update --exclude=.git --exclude=build . build/$TARGET/src/$(basename $ROOT_DIR)

# Do the build
pushd build/$TARGET
$ROOT_DIR/tools/docker/$TARGET/run.sh ${@:2}
popd
