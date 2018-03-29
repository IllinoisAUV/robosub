# Running Ardupilot Simulation with Gazebo

Dependencies:
```
sudo apt-get install -y ros-kinetic-desktop-full ros-kinetic-mavros
sudo pip install opencv_python numpy gi gobject PyYAML
```

Download ArduPilot and build the latest stable release
```sh
git clone https://github.com/ardupilot/ardupilot
cd ardupilot
git checkout -b ArduSub-stable

# Download and install build prerequisites
./Tools/scripts/install-prereqs-ubuntu.sh
```

Next, download the needed ROS packages
```sh
cd ~/catkin_ws/src
git clone https://github.com/patrickelectric/bluerov_ros_playground
git clone https://github.com/freefloating-gazebo/freefloating_gazebo
git clone https://github.com/freefloating-gazebo/freefloating_gazebo_demo
```


Build the code
```
cd ~/catkin_ws
catkin_make
```


Now, run the ArduPilot SITL (Make sure you're using python2)
```
cd Ardupilot/ArduSub
../Tools/autotest/sim_vehicle.py -j4
```

Now you can roslaunch motion and tell it that it is running in simulation
```
roslaunch robosub motion.launch sitl:=true
```
