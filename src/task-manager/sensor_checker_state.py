import rospy

from geometry_msgs.msg import Accel
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock



class Sensor_Checks(smach.State):
    def __init__(self, time_out):
        smach.State.__init__(self, outcomes=['Checks_passed', 'Checks_failed'])
        # rospy.init_node('sensor checker', anonymous=True)

        self.Imu_check = False
        self.Pressure_sensor_check = False

        self.timeout = time_out
        self.done = False
        self.sensor_check_complete = False

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
        self.sensor_check_complete = self.Pressure_sensor_check and self.Imu_check

        while( not self.sensor_check_complete and time < self.timeout):
            # @TODO read the topic names from launch file

            # checking imu data
            rospy.Subscriber("/rexrov/imu", Imu, self.Imucallback)

            # checking pressure data
            rospy.Subscriber("/rexrov/pressure", FluidPressure, self.Pressurecallback )

            rospy.spin()

            # sleep for a sec
            rospy.sleep(1)

            time += 1

        self.done = True
        return

        # imu subscriber callback
    def Imu_callback(self, data):
        self.Imu_check = True

        # pressure subscriber callback
    def Pressure_callback
        self.Pressure_sensor_check = True
