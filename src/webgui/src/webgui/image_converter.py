#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
import rospy
import cv2

class image_converter:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('image_topic',Image,self.callback, queue_size=10)
		
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, '8UC3')
		except CvBridgeError as re:
			rospy.logerr('Failed to convert img msg to cv2:\n%s', e)


def main(args):
	ic = image_converter()
	rospy.init_node('image_converter')
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down')
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
