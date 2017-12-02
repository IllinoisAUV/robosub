#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time


class image_pub:
	""" Class for publishing images from video source

	Attributes:
		pub (rospy.Publisher): Publisher that publishes images to topic.
		bridge (CvBridge): CvBridge instance that converts cv2 images to imgmsg
		cam (cv2.VideoCapture): Gets frames from video source
	"""
	def __init__(self, image_topic, video_source):
		"""Init method for image_pub
		
		Args:
			image_topic (str): Topic to publish to.
			video_source (str): Path to video file

		"""
		self.pub = rospy.Publisher(image_topic, Image, queue_size=10)
		self.bridge = CvBridge()
		self.cam = cv2.VideoCapture(video_source)
	
	def publish_images(self):
		"""Retrieves and publishes images from video source"""
		
		while self.cam.isOpened() and not rospy.is_shutdown():
			meta, frame = self.cam.read()
			msg_frame = self.bridge.cv2_to_imgmsg(frame)		
			self.pub.publish(msg_frame)

def main(args):
	# get params from rosparam
	node_name = 'imgpub'
	image_topic = rospy.get_param('imgpub/topic', 'l_cam_topic')
	video_source = rospy.get_param('imgpub/video')
	
	rospy.init_node(node_name)
	ic = image_pub(image_topic, video_source)
	try:
		ic.publish_images()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

	

if __name__ == '__main__':
	main(sys.argv)

