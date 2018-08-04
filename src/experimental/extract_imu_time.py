import tf.transformations as tf_transform
import time
import rospy

from sensor_msgs.msg import Imu

i = 0
start_time = time.time()

def callback(msg):
    if i % 1000 == 0:
        euler = tf_transform.euler_from_quaternion(msg.data.orientation)
        print('Time: %f'.format(time.time() - start_time))
        print('IMU: Roll(%f) Pitch(%f) Yaw(%f)'.format(euler[0], euler[1], euler[2]))

def main():
    imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, callback)
    rospy.spin()

if _name_ == '_main_':
    main()
