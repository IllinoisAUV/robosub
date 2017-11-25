#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

def cvt_imgmsg_to_cv2(data, bridge):
	try:
		cv2_img = bridge.imgmsg_to_cv2(data, '8UC3')
		ret, jpeg = cv2.imencode('.jpg', cv2_img)
		if jpeg is not None:
			return (b'--frame\r\n Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
		else:
			rospy.logerr('Failed to convert img msg to cv2:\n%s', 'Jpeg is none')

	except CvBridgeError as re:
		rospy.logerr('Failed to convert img msg to cv2:\n%s', e)
	
def video_feed(topic):
	"""Video streaming route. Put this in the src attribute of an img tag."""
	bridge = CvBridge()
	while True:
		data = rospy.wait_for_message(topic, Image)
		jpeg = cvt_imgmsg_to_cv2(data, bridge)
		yield(jpeg)
