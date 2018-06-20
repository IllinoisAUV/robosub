#!/usr/bin/env python

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robosub.msg import VisualTarget

class image_converter:
    
    def __init__(self):
    #publish ans subscribe targets
        self.pub = rospy.Publisher(rospy.get_param("~target"), VisualTarget, queue_size=1)

        self.bridge = CvBridge()

        
        self.image_sub = rospy.Subscriber(rospy.get_param('~input'), Image, self.callback)

    #process img and return center as target
    def process(self, img):
        cv2.cvtColor(img, cv2.COLOR_BGR2HSV, img)
        cv2.cvtColor(img, cv2.COLOR_HSV2BGR, img)
        
        #TODO: hard-coded for now, will change later
        target = VisualTarget()
        target.x = 10
        target.y = 50
        return target
        
    #call process on img to target
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        
        target_ctr = VisualTarget()
        target_ctr = self.process(cv_image)
        self.pub.publish(target_ctr)
        
        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

#subscribe and publish target msg
def main(args):
    rospy.init_node('image_processor', anonymous=True)
    ic = image_converter()
    
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
