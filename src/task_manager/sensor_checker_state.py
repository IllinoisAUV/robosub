#!/usr/bin/env python
import rospy

import smach
import smach_ros

from geometry_msgs.msg import Accel
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

import signal

class TimeoutException(Exception):   # Custom exception class
    pass

def timeout_handler(signum, frame):   # Custom signal handler
    a = signum
    b = frame
    raise TimeoutException

class Sensor_Checks(smach.State):
    def __init__(self, time_out):
        smach.State.__init__(self, outcomes=['Checks_passed', 'Checks_failed'])
        # rospy.init_node('sensor checker', anonymous=True)

        self.imu_check = False
        self.pressure_sensor_check = False

        self.timeout = time_out
        self.done = False
        self.sensor_check_complete = False
        signal.signal(signal.SIGALRM, timeout_handler)

        self.listener()

    def execute(self, userdata):
        # do all the checks
        rospy.loginfo("Sensor_Checks")

        # wait for check to succeed or timeout
        # since this will be for very short time we dont need action state
        while (not self.done):
            pass

        if(self.sensor_check_complete):
            rospy.loginfo("Checks_passed")
            return 'Checks_passed'

        else:
            rospy.loginfo("Checks_failed")
            return 'Checks_failed'

    def listener(self):
        # try to get Imu and pressure sensor value for 10 sec
        time = 0

        self.sensor_check_complete = (self.pressure_sensor_check and self.imu_check)

        while( not self.sensor_check_complete and time < self.timeout):

            signal.alarm(2)

            try:
                # checking imu ps
                rospy.Subscriber("/sm/imu", Imu, self.imu_callback)

                # checking pressure data
                rospy.Subscriber("/sm/pressure", FluidPressure, self.pressure_callback )
            except TimeoutException:
                continue # continue the while loop if we dont get message for 1 sec
            else:
                # Reset the alarm
                signal.alarm(0)

            # sleep for a sec
            rospy.sleep(1)

            time += 1

        self.sensor_check_complete = self.pressure_sensor_check and self.imu_check
        self.done = True
        return

        # imu subscriber callback
    def imu_callback(self, data):
        self.imu_check = True
        return

        # pressure subscriber callback
    def pressure_callback(self, data):
        self.pressure_sensor_check = True
        return
