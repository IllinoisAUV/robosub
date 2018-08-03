#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import TwistStamped
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8

class dice_state(object):
    def __init__(self):
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
        self.des_vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=1)
        self.num_sub = rospy.Subscriber("/darknet_ros/found_object",Int8, self.num_callback)
        # self.image_sub = rospy.Subscriber("/zed/rgb/image_rect_color",Image,self.img_callback)

        self.linear_speed = 0.2
        self.k_alt = 0.0005
        self.k_yaw = 0.0005

        self.idx = None
        self.center_x = None
        self.center_y = None

        self.h = 720
        self.w = 1280
        self.detected = False

    def num_callback(self, msg):
        self.detected_any = False
        if msg.data > 0:
            self.detected_any = True
        self.target_follower()

    def go_forward(self):
        while( not self.detected):
            self.target_follower()
        return

    def callback(self, msg):
        self.detected = False
        for i in range(len(msg.bounding_boxes)):
            if msg.bounding_boxes[i].Class == 'D6':
                self.idx = i
                self.detected = True
        if self.detected:
            x_min = msg.bounding_boxes[self.idx].xmin
            x_max = msg.bounding_boxes[self.idx].xmax

            y_min = msg.bounding_boxes[self.idx].ymin
            y_max = msg.bounding_boxes[self.idx].ymax

            self.center_x = (x_max + x_min)/2
            self.center_y = (y_max + y_min)/2
            print(self.center_x, self.center_y)
        # self.target_follower()

    def target_follower(self, msg=None):
        msg = TwistStamped()

        if self.detected and self.detected_any:
            d_alt = self.k_alt*(self.center_y - self.h/2)
            d_yaw = self.k_yaw*(self.center_x - self.w/2)

            msg.twist.linear.x = self.linear_speed
            msg.twist.linear.z = d_alt
            msg.twist.angular.z = d_yaw
        else:
            msg.twist.linear.x = self.linear_speed

        print("Message")
        print(msg)
        self.des_vel_pub.publish(msg)

def main(args):
    rospy.init_node('dice_state', anonymous=True)
    ec = dice_state()
    ec.go_forward()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
