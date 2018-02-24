import rospy

from geometry_msgs.msg import Accel
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

Imu_check = False
Pressure_sensor_check = False


class sensor_checker():
    def __init__(self):
        rospy.init_node('sensor checker', anonymous=True)

    def listener(self):

        # try to get Imu and pressure sensor value for 10 sec
        time = 0
        while( ((not sensor_checker.Pressure_sensor_check) and (not sensor_checker.Imu_check)) and time < 10):
            # @TODO read the topic names from launch file
            # checking imu data
            rospy.Subscriber("/rexrov/imu", Imu, self.Imucallback)

            # checking pressure data
            rospy.Subscriber("/rexrov/pressure", FluidPressure, self.Pressurecallback )

            rospy.spin()

            # sleep for a sec
            rospy.sleep(1)

            time += 1
        return

        # imu subscriber callback
    def Imu_callback(self, data):
        Imu_check = True

        # pressure subscriber callback
    def Pressure_callback
        Pressure_sensor_check = True
