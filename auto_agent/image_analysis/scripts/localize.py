import sys
import rospy
import cv2
import numpy as np
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

class localize():
	
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.color_detect)

	def color_detect(self, image_callback):
		print "calling back"
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image_callback, "bgr8")
		except CvBridgeError as e:
			print (e)
		image_callback = cv2.cvtColor(image_callback, cv2.COLOR_BGR2HSV)
		minRedVals = np.array([150,120,100], dtype=np.uint8)
		maxRedVals = np.array([180,255,255], dtype=np.uint8)
		redMask = cv2.inRange(image_callback, minRedVals, maxRedVals)
		red_image = cv2.bitwise_and(cv_image,cv_image, mask = redMask)
		rospy.Publisher('/red_image', red_image)
def main(args):
	ic = localize()
	rospy.init_node('localize', anonymous=True)
	try:
		print "three"
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
