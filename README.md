# IllinoisAUV Code
[![Build Status](https://travis-ci.org/IllinoisAUV/robosub.svg?branch=master)](https://travis-ci.org/IllinoisAUV/software)


## Building the code to run locallly
Run `catkin_make` in the repository root directory

## Running the tests
Run `catkin_make run_tests` in the repository root directory

## CI

Continuous integration is run on [Travis CI](https://travis-ci.org), through the free open source plan. Since Travis does not support Ubuntu 16.04, we have to rely on docker containers for installing kinetic. This also gives us a way to control the environment and what is installed.

CI will check that the code builds and the tests pass. In addition, it will run linters on the code.


## Style
We use clang-format to enforce style in the C++ files. You can format all of your code using `catkin_make --make-args clangformat`. It will automatically format all of your .cpp and .h files.

## Running the web UI

First, install the dependencies
```
pip install -r src/webgui/requirements.txt
```

Next, build the repository
```
catkin_make
```

Source the setup file
```
source devel/setup.bash
```

IN ANOTHER TERMINAL start roscore
```
roscore
```

Run the code
```
rosrun webgui run_server
```
