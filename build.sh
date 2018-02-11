#!/bin/bash
set -e
ROOT_DIR=$(dirname $(readlink -f $0))

TARGET=$1
case $TARGET in
    host);;
    jetson);;
    clean)
        rm -rf build/*
        exit 0
        ;;
    *)
        echo "Please specify a build target among [host, jetson, test]"
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
