#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

class dice_state(object):
    def __init__(self):
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
        self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=1)
        self.linear_speed = 0.4
        self.k_alt = 0.005
        self.k_yaw = 0.005

        self.idx
        self.center_x = None
        self.center_y = None

        self.h = None
        self.w = None

    def callback(self, msg):
        for i in range(4):
            if msg.bounding_boxes[i].Class == 'D6':
                self.idx = i
        x_min = msg.bounding_boxes[self.idx].xmin
        x_max = msg.bounding_boxes[self.idx].xmax

        y_min = msg.bounding_boxes[self.idx].ymin
        y_max = msg.bounding_boxes[self.idx].ymax

        self.center_x = (x_max - x_min)/2
        self.center_y = (y_max - y_min)/2
        print(self.center_x, self.center_y)

    def target_follower(self):

        msg = Twist()
        d_alt = self.k_alt*(self.h/2 - self.center_y)
        d_yaw = self.k_yaw*(self.w/2 - self.center_x)

        msg.linear.x = self.linear_speed_x
        msg.linear.z = d_alt
        msg.angular.z = d_yaw
        print("Message")
        print(msg)
        self.des_vel_pub.publish(msg)

def main(args):
  ec = dice_state()
  rospy.init_node('dice_state', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
