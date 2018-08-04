#!/usr/bin/env python
import rospy
import sys

import cv2
import time

from geometry_msgs.msg import TwistStamped
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8

class dice_state(object):
    def __init__(self):
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
        self.des_vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=1)
        self.num_sub = rospy.Subscriber("/darknet_ros/found_object",Int8, self.num_callback)
        # self.image_suurn_time = self.start_time + selfb = rospy.Subscriber("/zed/rgb/image_rect_color",Image,self.img_callback)

        self.YAW_TURN_SPEED = -0.2
        self.GATE_STRAIGHT_TIME = 20.0
        self.GATE_ALT_TIME = 5.0
        self.TURN_TIME = 2.0

        self.idx = None
        self.center_x = None
        self.center_y = None

        self.start_time = time.time()
        self.depth_time = self.start_time + self.GATE_ALT_TIME
        self.forward_time = self.depth_time + self.GATE_STRAIGHT_TIME
        self.turn_time = self.forward_time + self.TURN_TIME

        self.h = 720
        self.w = 1280
        self.detected = False
        self.gate_detected = False

    def go_down(self):
        msg = TwistStamped()
        msg.twist.linear.z = -0.4
        self.des_vel_pub.publish(msg)

    def go_straight(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.4
        self.des_vel_pub.publish(msg)

    def turn(self):
        msg = TwistStamped()
        msg.twist.angular.z = self.YAW_TURN_SPEED
        self.des_vel_pub.publish(msg)

    def execute(self):
        while(time.time() < self.start_time + self.GATE_ALT_TIME):
            self.go_down()
        while (time.time() < self.depth_time + self.GATE_STRAIGHT_TIME):
            self.go_straight()
        while(time.time() < self.forward_time + self.TURN_TIME):
            self.turn()
        self.DONE = True

    def num_callback(self, msg):
        if(self.DONE):
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

    def gate_follower(self, msg=None):
        msg = TwistStamped()

        if self.gate_detected:
            d_alt = self.k_alt*(self.gate_center_y - self.h/2)
            d_yaw = self.k_yaw*(self.gate_center_x - self.w/2)

            msg.twist.linear.x = self.linear_speed
            msg.twist.linear.z = d_alt
            msg.twist.angular.z = d_yaw
        else:
            msg.twist.linear.x = self.linear_speed

        print("Gate Servoing:")
        print(msg)
        self.des_vel_pub.publish(msg)

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

        print("Dice Message")
        print(msg)
        self.des_vel_pub.publish(msg)

def main(args):
    rospy.init_node('dice_state', anonymous=True)
    ds = dice_state()
    ds.execute()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
