#!/usr/bin/env python
import rospy

from config import Config
from mover import Mover

class VisualServo(Task):
    timeout = rospy.Duration(0.5)

    def __init__(self, image_dims):
        # Target is a tuple of (x,y) that specifies the point at which the
        # Target is at on the screen in pixels
        self.target = (0,0)
        self.last_target_time = 0

        # dims is a tuple representing the (width, height) of the image in pixels
        self.dims = image_dims

        # Time out the visual servoing
        self.timer = rospy.Timer(self.timeout, self.timeout, oneshot=True)

        # Start in the timed out state
        self.timed_out = True
        self.started = False
        self.done = False

    def run(self):
        self.started = True
        r = rospy.Rate(10)
        while not self.done and not rospy.is_shutdown():
            r.sleep()
    
    def restart_timeout(self):
        self.timed_out = False
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timeout)

    def timeout(self):
        self.timed_out = True
        self.timer.shutdown()

    def follow_target(self):
        if self.timed_out:
            # No targets, just go straight
            d_yaw = 0.0
            d_alt = 0.0
        else:
            d_yaw = Config.visual_servo_kp_yaw * (self.target[0] - self.dims[0]/2)
            d_alt = Config.visual_servo_kp_alt * (self.target[1] - self.dims[1]/2)

        msg = TwistStamped()
        msg.linear.x = Config.visual_servo_forward_speed
        msg.linear.z = d_alt
        msg.angular.z = d_yaw
        self.mover.publish(msg)

        # TODO: Use jerk to detect hitting a target


class DarknetVisualServo(VisualServo):
    def __init__(self, bbox_topic, image_dims, label):
        self.bbox_sub = rospy.Subscriber(bbox_topic, BoundingBoxes, self.callback)
        self.label = label

    def callback(self, msg):
        idx = -1
        for i in range(len(msg.bounding_boxes)):
            # Take the first matching detection
            if msg.bounding_boxes[i].Class == self.label:
                idx = i
                break

        if idx == -1:
            return

        # Find center of the detection
        self.restart_timeout()
        x_min = msg.bounding_boxes[idx].xmin
        x_max = msg.bounding_boxes[idx].xmax

        y_min = msg.bounding_boxes[idx].ymin
        y_max = msg.bounding_boxes[idx].ymax

        center_x = (x_max + x_min)/2
        center_y = (y_max + y_min)/2
        self.target = (center_x, center_y)
        self.last_target_time = time.time()

        self.follow_target()
