import numpy as np
import cv2

class HsvFilter(object):

    def __init__(self, lowHue, lowSat, lowVal, highHue, highSat, highVal):
        self.minV = np.array([lowHue, lowSat, lowVal], np.uint8)
        self.maxV = np.array([highHue, highSat, highVal], np.uint8)

    def process(self, img):
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_image_thres = cv2.inRange(hsv_image, self.minV, self.maxV)
        return hsv_image_thres
