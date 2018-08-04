#!/usr/bin/env python
import rospy


from mover import Mover
from gate import Gate
from visual_servo import DarknetVisualServo
from config import Config




if __name__ == "__main__":
    rospy.init_node('main', anonymous=True)

    
    mover = Mover()
    armer = Armer()
    gate = Gate()
    dice = DarknetVisualServo(Config.darknet_topic, zed_camera_dims, 'D6')

    armer.arm()
    gate.run()
    mover.turn(Config.dice_yaw_time, Config.dice_yaw_speed)
    dice.run()

