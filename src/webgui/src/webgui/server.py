#!/usr/bin/env python
from flask import Flask, render_template, Response
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import rospkg
import threading
import cv2


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
    return render_template('index.html', sub=sub)

# @app.route("/l_camera")
def cvt_imgmsg_to_cv2(data, bridge):
	try:
		cv2_img = bridge.imgmsg_to_cv2(data, '8UC3')
		ret, jpeg = cv2.imencode('.jpg', cv2_img)
		if jpeg is not None:
			return (b'--frame\r\n Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
		else:
			rospy.logerr("frame is none")


	except CvBridgeError as re:
		rospy.logerr('Failed to convert img msg to cv2:\n%s', e)
	
def video_feed():
	"""Video streaming route. Put this in the src attribute of an img tag."""
	bridge = CvBridge()
	while True:
		rospy.logdebug('Waiting for img')
		data = rospy.wait_for_message('image_topic', Image)
		jpeg = cvt_imgmsg_to_cv2(data, bridge)
		yield(jpeg)
		
		

@app.route('/l_camera')
def l_camera():
	return Response(video_feed(), mimetype='multipart/x-mixed-replace; boundary=frame')
                    
# Sample callback for ROS to use
def callback(data):
    print(data.data)


def main():
    if os.environ.get('WERKZEUG_RUN_MAIN') == 'true':
        # When running flask with debug=True, rospy has to be set up in this if
        # guard because the flask reloader forks and then calls the server, 
        # which will be killed and restarted on a reload. This causes some 
        # problems with rospy that I haven't been able to debug
        # See https://stackoverflow.com/questions/25504149/why-does-running-the-flask-dev-server-run-itself-twice
        rospy.init_node('webserver', log_level=rospy.DEBUG,  disable_signals=True)
        rospy.Subscriber('testing', String, callback)
        rospy.Subscriber("testing2", String, callback)
        rospy.Subscriber('l_camera', Image, video_feed)
        
    # We do not need to run rospy.spin() here because all rospy.spin does is
    # block until the node is supposed to be killed
    app.run(host="0.0.0.0", debug=True)
