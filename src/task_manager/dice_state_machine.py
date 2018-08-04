#!/usr/bin/env python
import rospy
import sys

import cv2
import time

from geometry_msgs.msg import TwistStamped
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8, Float32

class dice_state(object):
    def __init__(self):
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
        self.des_vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=1)
        self.num_sub = rospy.Subscriber("/darknet_ros/found_object",Int8, self.num_callback)
        self.jerk_sub= rospy.Subscriber("/jerk", Float32, self.jerk_callback )
        # self.image_suurn_time = self.start_time + selfb = rospy.Subscriber("/zed/rgb/image_rect_color",Image,self.img_callback)

        self.YAW_TURN_SPEED = 0.2
        self.GATE_STRAIGHT_TIME = 27.0

        self.FORWARD_TIME = 10.0

        self.GATE_ALT_TIME = 7.0
        self.TURN_TIME = 0.7
        self.TURN_180_TIME = 1.5

        self.GO_DOWN_2_TIME = 2.0

        self.linear_speed = 0.2
        self.INIT_MOVT_DONE = False

        self.JERK_THRESH = 1.5

        self.idx = None
        self.center_x = None
        self.center_y = None

        self.start_time = time.time()
        self.depth_time = self.start_time + self.GATE_ALT_TIME
        self.forward_time = self.depth_time + self.GATE_STRAIGHT_TIME
        self.turn_time = self.forward_time + self.TURN_TIME

        self.k_alt = 0.003
        self.k_yaw = 0.001
        self.DONE = False

        self.state = 1

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

    def go_backward(self):
        msg = TwistStamped()
        msg.twist.linear.x = -0.4
        self.des_vel_pub.publish(msg)

    def turn(self):
        msg = TwistStamped()
        msg.twist.angular.z = self.YAW_TURN_SPEED
        self.des_vel_pub.publish(msg)

    def execute(self):
        i = 0
        while(time.time() < self.start_time + self.GATE_ALT_TIME and not
                rospy.is_shutdown()):
            if i % 1000 == 0:
                print("Diving")
            self.go_down()
            i += 1
        while (time.time() < self.depth_time + self.GATE_STRAIGHT_TIME and not
                rospy.is_shutdown()):
            if i % 1000 == 0:
                print("Going straight")
            self.go_straight()
            i += 1
        while(time.time() < self.forward_time + self.TURN_TIME and not
                rospy.is_shutdown()):
            if i % 1000 == 0:
                print("Turning")
            self.turn()
            i += 1

        self.INIT_MOVT_DONE = True

    def execute_2(self):

        print("Dice Hit")
        self.INIT_MOVT_DONE = False


        while(time.time() < forward_time_2 and not rospy.is_shutdown()):
            if i % 1000 == 0:
                print("Forward 2")
            self.go_forward()
            i += 1

        forward_time_2 = time.time() + self.FORWARD_TIME_2
        while(time.time() < forward_time_2 and not rospy.is_shutdown()):
            if i % 1000 == 0:
                print("Forward 2")
            self.go_forward()
            i += 1

        turn_180 = time.time() + self.TURN_180_TIME

        while(time.time() < turn_180  and not rospy.is_shutdown() ):
            if i % 1000 == 0:
                print("Turing 180")
            self.turn()

        go_down_2 = time.time() + self.GO_DOWN_2_TIME

        while(time.time() < go_down_2  and not rospy.is_shutdown() ):
            if i % 1000 == 0:
                print("Go down 2")
            self.go_down()()

        self.INIT_MOVT_DONE = True
        self.state = 2

    def num_callback(self, msg):
        if(self.INIT_MOVT_DONE):
            self.detected_any = False
            if msg.data > 0:
                self.detected_any = True
            print("num_callback target_follower")
            self.target_follower()

    def callback(self, msg):
        target_dice = ''
        if self.state == 1:
            target_dice = 'D6'
        elif self.state == 2:
            target_dice = 'D5'

        self.detected = False
        for i in range(len(msg.bounding_boxes)):
            if msg.bounding_boxes[i].Class == target_dice:
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

    def jerk_callback(self, msg):
        jerk = msg.data
        if jerk > self.JERK_THRESH and not self.dice_hit:
            self.dice_hit = True
            self.execute_2()


    def target_follower(self):
        msg = TwistStamped()

        if self.detected and self.detected_any:
            d_alt = self.k_alt*(self.center_y - self.h/2)
            d_yaw = self.k_yaw*(self.center_x - self.w/2)

            msg.twist.linear.x = self.linear_speed
            msg.twist.linear.z = d_alt
            msg.twist.angular.z = d_yaw
        else:
            msg.twist.linear.x = self.linear_speed

        print("Target following")
        print(msg)
        self.des_vel_pub.publish(msg)

def main(args):
    rospy.init_node('dice_state', anonymous=True)
    ds = dice_state()
    ds.INIT_MOVT_DONE = True
    # ds.execute()
    # ds.execute_2()
    print("Starting gate")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
