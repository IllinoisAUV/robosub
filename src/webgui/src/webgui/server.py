#!/usr/bin/env python

from flask import Flask, render_template
import rospy
from std_msgs.msg import String
import os
import sys
import rospkg
import threading


# This is needed in order to find templates, config and static directories due
# to how catkin handles python file installation. Since it will `exec` this 
# file from a different location, we must use the ROS API's to locate the real
# instance_path
rp = rospkg.RosPack()
app = Flask(__name__, instance_path=os.path.join(rp.get_path("webgui"), "src", "webgui"))


@app.route("/")
def index():
    return render_template('index.html')

def callback(data):
    print(data.data)

def main():
    rospy.Subscriber('testing', String, callback)

    rospy.init_node('webserver', disable_signals=True)

    # We do not need to run rospy.spin() here because all rospy.spin does is
    # block until the node is supposed to be killed
    app.run(host="0.0.0.0", debug=True)

