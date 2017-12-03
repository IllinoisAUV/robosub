#!/usr/bin/env python
from flask import Flask, render_template, Response
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import os
import sys
import rospkg

class Sub:
    def __init__(self, name):
        self.name = name

sub = Sub('Enigma')

# This is needed in order to find templates, config and static directories due
# to how catkin handles python file installation. Since it will `exec` this 
# file from a different location, we must use the ROS API's to locate the real
# instance_path
rp = rospkg.RosPack()
app = Flask(__name__, instance_path=os.path.join(rp.get_path("webgui"), "src", "webgui"))


@app.route("/")
def index():
    url = rospy.get_param('/web_video_server/address')
    port = rospy.get_param('/web_video_server/port')
    stream_server = ('http://%s:%s' % (url, port))
    print(rospy.get_param('/camera/bottom'),)
    return render_template('index.html', sub=sub, stream_server=stream_server, 
                           bottom_camera=rospy.get_param('/camera/bottom'),
                           front_camera=rospy.get_param('/camera/front'))
def main():
    if os.environ.get('WERKZEUG_RUN_MAIN') == 'true':
        # When running flask with debug=True, rospy has to be set up in this if
        # guard because the flask reloader forks and then calls the server, 
        # which will be killed and restarted on a reload. This causes some 
        # problems with rospy that I haven't been able to debug
        # See https://stackoverflow.com/questions/25504149/why-does-running-the-flask-dev-server-run-itself-twice
        rospy.init_node('webserver', log_level=rospy.DEBUG,  disable_signals=True)
        
    # We do not need to run rospy.spin() here because all rospy.spin does is
    # block until the node is supposed to be killed
    app.run(host="0.0.0.0", debug=True, threaded=True)
