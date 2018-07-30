#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    pub = rospy.Publisher('output', Twist, queue_size=1)
    def callback(msg):
        output = Twist()
        output.linear.x = 0.0
        output.linear.y = 0.0
        output.linear.z = msg.data

        pub.publish(output)

    sub = rospy.Subscriber('input', Float64, callback)
    rospy.init_node('dive_effort_to_twiststamped', anonymous=True)

    rospy.spin()
