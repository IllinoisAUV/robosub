# IllinoisAUV Code
[![Build Status](https://travis-ci.org/IllinoisAUV/robosub.svg?branch=master)](https://travis-ci.org/IllinoisAUV/software)


## Building the code to run locallly
Run `catkin_make` in the catkin workspace

## Running the tests
Run `catkin_make run_tests` in the repository root directory


## Style
We use clang-format to enforce style in the C++ files. You can format all of
your code using `catkin_make --make-args clang-format` in the catkin workspace 
or `./build.sh clang-format`. It will automatically format all of your .cpp and
.h files.


# Building with Docker
If you have the docker daemon running, you can just run `./build.sh <target>`,
where target is either `host` or `jetson`. It will build your code in
`build/<target>`.  Docker is a useful tool that lets you run "containers" on any
computer. If you are on Mac or Linux, and have the docker daemon running,
`build.sh` will build your code correctly. `build.sh` has a few other commands,
which are documented in its `-h` option.

## CI

Continuous integration is run on [Travis CI](https://travis-ci.org), through the
free open source plan. CI will check that the code builds and the tests pass. In
addition, it will run linters on the code.

