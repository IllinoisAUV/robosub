#!/usr/bin/env python
import rospy
import sys

import cv2

from geometry_msgs.msg import TwistStamped
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8

class dice_state(object):
    def __init__(self):
        # self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
        self.des_vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=1)
        # self.num_sub = rospy.Subscriber("/darknet_ros/found_object",Int8, self.num_callback)
        self.image_sub = rospy.Subscriber("/zed/rgb/image_rect_color",Image,self.img_callback)

        self.linear_speed = 0.2
        self.k_alt = 0.0005
        self.k_yaw = 0.0005

        self.idx = None
        self.center_x = None
        self.center_y = None

        self.h = 720
        self.w = 1280
        self.detected = False
        self.gate_detected = False

    def extract_init_img(self, cv_frame):
		gate = cv2.imread('template.png')
		templ_gray = cv2.cvtColor(gate, cv2.COLOR_BGR2GRAY)
		frame_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
		self.h , self.w = frame_gray.shape
		tl, br = self.tem_match(cv_frame, frame_gray, templ_gray)
		self.gate_bbox = self.get_bbox(tl, br)
		# self.yellow_bbox = self.get_bbox(all_tl[self.yellow], all_br[self.yellow])
		# self.red_bbox = self.get_bbox(all_tl[self.red], all_br[self.red])

    def get_bbox(self,tl,br):
    	bbox = (tl[0], tl[1], br[0]-tl[0], br[1]-tl[1])
    	return bbox

    def tem_match(self, orig, src, templ):
		img = src
		img2 = img.copy()
		template = templ
		w, h = template.shape[::-1]

		tl = None
		br = None
		all_point = None

		point = None

		methods = ['cv2.TM_CCOEFF_NORMED']
		for meth in methods:
			img = img2.copy()
			resize_i = img2.copy()
			method = eval(meth)
			orig_res = None
			for i in range(2):
				resize_i = cv2.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv2.INTER_AREA)
				# Apply template Matching
				res = cv2.matchTemplate(resize_i, template, method)
				if i == 0:
				    orig_res = res
				threshold = 0.70
				loc = np.where( res >= threshold)
				first_point = 0
				for pt in zip(*loc[::-1]):
                    self.gate_detected = True
					point = pt
					# green
					if first_point == 0:
						all_point[0] = pt
						cv2.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,255,0), 1)

		#yellow
		tl = (all_point[0][0],all_point[0][1])
		br = ((all_point[0][0] + w),(all_point[0][1] + h))

		# self.center = center = ((br[0]-tl[0])/2 , (br[1]-tl[1])/2)
		# cv2.circle(orig,(tl[0] + center[0], tl[1] + center[1]), 1, (0,0,255), 2)
		cv2.imshow('Matching Result', orig_res)
		cv2.imshow('Detected Point', orig)
		cv2.waitKey(10)
		return tl, br

    def img_callback(self,data):
        self.gate_detected = True
        try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.extract_init_img(cv_image)

        self.gate_center_x = (self.gate_bbox[0] + self.gate_bbox[2])/2
        self.gate_center_y = (self.gate_bbox[1] + self.gate_bbox[3])/2
        self.gate_follower()
        cv2.waitKey(10)

    def num_callback(self, msg):
        self.detected_any = False
        if msg.data > 0:
            self.detected_any = True
        self.target_follower()

    def go_forward(self):
        while( not self.detected):
            self.target_follower()
        return

    def callback(self, msg):
        self.detected = False
        for i in range(len(msg.bounding_boxes)):
            if msg.bounding_boxes[i].Class == 'D6':
                self.idx = i
                self.detected = True
        if self.detected:
            x_min = msg.bounding_boxes[self.idx].xmin
            x_max = msg.bounding_boxes[self.idx].xmax

            y_min = msg.bounding_boxes[self.idx].ymin
            y_max = msg.bounding_boxes[self.idx].ymax

            self.center_x = (x_max + x_min)/2
            self.center_y = (y_max + y_min)/2
            print(self.center_x, self.center_y)
        # self.target_follower()

    def gate_follower(self, msg=None):
        msg = TwistStamped()

        if self.gate_detected:
            d_alt = self.k_alt*(self.gate_center_y - self.h/2)
            d_yaw = self.k_yaw*(self.gate_center_x - self.w/2)

            msg.twist.linear.x = self.linear_speed
            msg.twist.linear.z = d_alt
            msg.twist.angular.z = d_yaw
        else:
            msg.twist.linear.x = self.linear_speed

        print("Gate Servoing:")
        print(msg)
        self.des_vel_pub.publish(msg)

    def target_follower(self, msg=None):
        msg = TwistStamped()

        if self.detected and self.detected_any:
            d_alt = self.k_alt*(self.center_y - self.h/2)
            d_yaw = self.k_yaw*(self.center_x - self.w/2)

            msg.twist.linear.x = self.linear_speed
            msg.twist.linear.z = d_alt
            msg.twist.angular.z = d_yaw
        else:
            msg.twist.linear.x = self.linear_speed

        print("Dice Message")
        print(msg)
        self.des_vel_pub.publish(msg)

def main(args):
    rospy.init_node('dice_state', anonymous=True)
    ec = dice_state()
    ec.go_forward()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
