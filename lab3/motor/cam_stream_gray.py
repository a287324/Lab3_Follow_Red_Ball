#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CamStream:
	def __init__(self):
		rospy.init_node("CamStream", anonymous = False)
		rospy.Subscriber("cv_camera/image_raw", Image, self.imageCallback)
		self.pub = rospy.Publisher('CamStream/image', Image, queue_size = 1)
		self.bridge = CvBridge()
	
	def imageCallback(self, image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8") 			# Change to openCV Format
			cv2.imwrite('original.bmp', cv_image)
			print('shape = ', cv_image.shape)
			cv_image_blur = cv2.GaussianBlur(cv_image, (3,3), 0)			# Gaussian Blur
			cv_image_hsv = cv2.cvtColor(cv_image_blur, cv2.COLOR_BGR2HSV) 	# Color to Gray
			
			cv_image_hsv_split = cv2.split(cv_image_hsv)					# Histogram equalization
			cv_image_hsv_split[2] = cv2.equalizeHist(cv_image_hsv_split[2])
			cv_image_equalized = cv2.merge(cv_image_hsv_split)
			
			self.lower = np.array([ 0, 0, 0])								# Extract Target Color: Red
			self.upper = np.array([ 10, 255, 255])
			cv_image_binary = cv2.inRange(cv_image_equalized, self.lower, self.upper)
			
			
			
			morphology = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))	# 3*3 square
			cv_image_erode = cv2.erode(cv_image_binary, morphology)			# Erode
			cv_image_dilate = cv2.dilate(cv_image_erode, morphology)		# Dilate	
			
			#cv_roi_image = cv_image_hsv_split[2] & cv_image_binary
				
			#cv_image_canny = cv2.Canny(cv_roi_image, 70, 140, 3)			# Canny edge detection
			# Hough circle
			circles = cv2.HoughCircles(cv_image_dilate, cv2.HOUGH_GRADIENT, 1, minDist=20, 
							param1=140, param2=70, minRadius=3, maxRadius=300)
			print(circles)
			if circles is not None:
				for i in circles[0,:]:
					cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2) # Draw circle
					cv2.circle(cv_image, (i[0], i[1]), 2, (255, 0, 255), 2) # Draw center
					
					cv2.circle(cv_image_dilate, (i[0], i[1]), i[2], (0, 255, 0), 2) # Draw circle
					cv2.circle(cv_image_dilate, (i[0], i[1]), 2, (255, 0, 255), 2) # Draw center
			
			#ret, cv_image_thres = cv2.threshold(cv_image_equalized, 127, 	# Binary threshold
			#						255, cv2.THRESH_BINARY)
			#cv_image_canny = cv2.Canny(cv_image_equalized, 70, 140, 3)		# Canny edge detection
			#morphology = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))	# 3*3 square
			#cv_image_morphology = cv2.erode(cv_image_equalized, morphology)		# Erode
			
			
			image = self.bridge.cv2_to_imgmsg(cv_image_dilate, "mono8") # Change to ROS Format
			#cv2.imshow('roi_img', cv_roi_image)
			cv2.imshow('original_hough_img', cv_image)
			cv2.waitKey(1)
			self.pub.publish(image)
			#self.pub.publish(edge_img)
		except CvBridgeError as e:
			rospy.logerr(e)


if __name__ == '__main__':
	try:
		cs = CamStream()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
