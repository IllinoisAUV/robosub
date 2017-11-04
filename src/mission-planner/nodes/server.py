#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.cfg import mission_planner

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("mission_planner", anonymous = True)

    srv = Server(mission_planner, callback)
    rospy.spin()
