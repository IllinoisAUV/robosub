#!/usr/bin/env python
import rospy

import smach
import smach_ros

from geometry_msgs.msg import Accel
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock



class Sensor_Checks(smach.State):
    def __init__(self, time_out):
        smach.State.__init__(self, outcomes=['Checks_passed', 'Checks_failed'])
        # rospy.init_node('sensor checker', anonymous=True)

        self.imu_check = False
        self.pressure_sensor_check = False

        self.timeout = time_out
        self.done = False
        self.sensor_check_complete = False
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

        while( not (self.Pressure_sensor_check and self.Imu_check) and time < self.timeout):
            # @TODO read the topic names from launch file

            # checking imu ps
            rospy.Subscriber("/rexrov/imu", Imu, self.Imu_callback)

            # checking pressure data
            rospy.Subscriber("/rexrov/pressure", FluidPressure, self.Pressure_callback )

            # rospy.spin()
            # sleep for a sec
            rospy.sleep(1)

            time += 1

        self.sensor_check_complete = self.Pressure_sensor_check and self.Imu_check
        self.done = True
        return

        # imu subscriber callback
    def Imu_callback(self, data):
        print("IMU message recieved")
        self.Imu_check = True
        return

        # pressure subscriber callback
    def Pressure_callback(self, data):
        print("pressure message recieved")
        self.Pressure_sensor_check = True
        return
