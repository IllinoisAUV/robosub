from geometry_msgs.msg import TwistStamped
import rospy
import time

from config import Config

class Mover(object):
    hz = 10
    def __init__(self):
        self.pub = rospy.Publisher(Config.mover_topic, TwistStamped, queue_size=1)
        self.hz = 10

    def _common_end_(self):
        if(rospy.is_shutdown()):
            return
        # Stop the sub once complete
        self.pub.publish(TwistStamped())

    def _send_message_duration_(self, msg, duration):
        end_time = duration + time.time() 
        while time.time() < end_time and not rospy.is_shutdown():
            self.pub.publish(msg)
            time.sleep(1.0/self.hz)
        self._common_end_()

    def dive(self, duration, speed=-0.4):
        msg = TwistStamped()
        msg.twist.linear.z = speed
        self._send_message_duration_(msg, duration)

    def forward(self, duration, speed=0.4):
        msg = TwistStamped()
        msg.twist.linear.x = speed
        self._send_message_duration_(msg, duration)

    def turn(self, duration, speed):
        msg = TwistStamped()
        msg.twist.linear.x = speed
        self._send_message_duration_(msg, duration)

    def publish(twist_stamped):
        self.pub.publish(twist_stamped)
