#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class PathPicker:

	def __init__(self, image_file):
        img_BGR = np.asarray(cv.imread(image_file))
        black_white = cv.cvtColor(img_BGR, cv.COLOR_BGR2GRAY)
        # finding edges (new image array)
        edges = cv.Canny(black_white, lower, upper)
        # finding contours of edge image (list of image arrays)
        image, contours, hierarchy = cv.findContours(edges, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)

        self.contours = {i : contours for i in range(len(contours))}
		self.pub = rospy.Publisher("path_assignments", Path)
		self.sub = rospy.Subscriber("path_requests", Request, callback)

	def callback(request, self.contours):
		x_coords, y_coords = self.find_path(request)
        self.pub.publish(Path(x = x_coords, y = y_coords))

    def find_path(request, contours):


if name == '__main__':
	path_assigner = PathPicker()
	rospy.init_node('path_assigner', anonymous = True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv.destroyAllWindows()
