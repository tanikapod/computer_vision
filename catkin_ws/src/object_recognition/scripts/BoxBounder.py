#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class BoxBounder:

	def __init__(self):
		self.pub = rospy.Publisher("box_topic", Point)
		self.sub = rospy.Subscriber("box_topic", Image, callback)

	def callback(self, image):
		try:
			cv_image = CvBridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError:
			print("Error converting image format")

		x, y, w, h = box_blue(cv_image)
		self.pub.publish(Point(x = x, y = y, z = 0))

	def box_blue(original_image):
	    # find color range
	    blue = np.uint8([[[255, 0, 0 ]]])
	    hsv_blue = cv.cvtColor(blue, cv.COLOR_BGR2HSV)
	    lower_blue = np.array([hsv_blue[0][0][0] - 50, 50, 50]) # in HSV
	    upper_blue = np.array([hsv_blue[0][0][0] + 50, 255, 255]) # in HSV

	    # find contours in color range
	    image_HSV = img.convert_to_HSV(original_image)
	    mask = cv.inRange(image_HSV, lower_blue, upper_blue)
	    image, contours, hierarchy = cv.findContours(mask, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)

	    # find rectangle bounding largest contour in color range
	    contour = max(contours, key = cv.contourArea)
	    x, y, w, h = cv.boundingRect(contour)

	    return x, y, w, h


if name == '__main__':
	box_finder = BoxBounder()
	rospy.init_node('box_bounder', anonymous = True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
