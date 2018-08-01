import rospy
import roslib
from geometry_msgs.msg import Twist
import time

# hackiest way to qualify
if __name__ == '__main__':
    rospy.init_node('qual_motion_controller', anonymous=True)
    sub_move_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # start after 30 sec
    time.sleep(10)

    rate = rospy.Rate(20) # 10hz
    msg_down = Twist()
    time_end_depth = time.time() + 6

    # going down for 5 sec
    while time.time() < time_end_depth:
        msg_down.linear.z = -0.5
        sub_move_pub_.publish(msg_down)
        rospy.loginfo("DOWN")
        rate.sleep()

    time_end_straight = time.time() + 4*60
    msg_staight = Twist()
    while time.time() < time_end_straight:
        msg_staight.linear.x = 0.7
        sub_move_pub_.publish(msg_staight)
        rospy.loginfo("STRAIGHT")
        rate.sleep()
