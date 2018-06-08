import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from robosub.msg import VisualTarget

class image_converter:

	#chk	VisualTarget compatibility
	
	def __init__(self):
	#chk what to init
		self.pub = rospy.Publisher("target", VisualTarget, queue_size=1)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

	def process(img):
		cv.CvtColor(img, img, CV_BGR2HSV)
		cv.CvtColor(img, img, CV_HSV2BGR)
		
		#chk py compatibility below
		
		target = VisualTarget()
		target.x = 10
		target.y = 50
		return target
		

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		
		target_ctr = VisualTarget()
		target_ctr = self.process(cv_image)
		self.pub.publish(target_ctr);
		
		
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)


	#dont need it just make subscriber
def main(args):
	rospy.init_node('image_processor', anonymous=True)
	ic = image_converter()
	#add nodehandle equiv?
	
	#sub = rospy.Subscriber("videofile/image_raw/", Image, callback)
	#pub = rospy.Publisher("target", VisualTarget, queue_size=1)
	
	rospy.spin()


if __name__ == '__main__':
		main(sys.argv)
