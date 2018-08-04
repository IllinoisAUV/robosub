#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

from config import Config

class Armer(object):
    def __init__(self):
        self.pub = rospy.Publisher(Config.arming_topic, Bool, queue_size=1)

    def arm(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)

    def disarm(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
