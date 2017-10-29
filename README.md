# IllinoisAUV Code
[![Build Status](https://travis-ci.org/IllinoisAUV/software.svg?branch=master)](https://travis-ci.org/IllinoisAUV/software)


## Building the code to run locallly
Run `catkin_make` in the repository root directory

## Running the tests
Run `catkin_make run_tests` in the repository root directory

## CI

Continuous integration is run on [Travis CI](https://travis-ci.org), through the free open source plan. Since Travis does not support Ubuntu 16.04, we have to rely on docker containers for installing kinetic. This also gives us a way to control the environment and what is installed.

For now, CI uses its own docker container, though we'd like to eventually merge all of the build systems into one.


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

Run the code
```
rosrun webgui run_server
```
