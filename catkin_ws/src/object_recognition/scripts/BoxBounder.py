import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_mgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import object_recognition

class BoxBounder:

	def __init__(self):
		self.pub = rospy.Publisher("box_topic", Point)
		self.sub = rospy.Subscriber("box_topic", Image, self.callback)

	def callback(self, image):
		try:
			cv_image = CvBridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError:
			print("Error converting image format")

		box = object_recognition.outline_blue(cv_image, False, "")

		self.pub.publish(box)


def main():
	box_finder = BoxBounder()
	rospy.init_node('box_bounder', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if name == '__main__':
	main()