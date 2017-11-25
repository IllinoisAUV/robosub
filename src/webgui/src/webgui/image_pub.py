#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time

node_name = rospy.get_param('imgpub/node_name', 'imgpub_node')
image_topic = rospy.get_param('imgpub/topic', 'l_cam_topic')
video = rospy.get_param('imgpub/video')


class image_pub:
	def __init__(self, args):
		self.image_pub = rospy.Publisher(image_topic, Image, queue_size=10)
    		self.bridge = CvBridge()
		self.cam = cv2.VideoCapture(video)
	
	def publish_images(self):
		while self.cam.isOpened() and not rospy.is_shutdown():
			meta, frame = self.cam.read()

			msg_frame = CvBridge().cv2_to_imgmsg(frame)		

			self.image_pub.publish(msg_frame)

def main(args):
	rospy.init_node(node_name)
	ic = image_pub(args)
	try:
		ic.publish_images()
	except KeyboardInterrupt:
		print("Shutting down")
 	cv2.destroyAllWindows()

	

if __name__ == '__main__':
    main(sys.argv)

