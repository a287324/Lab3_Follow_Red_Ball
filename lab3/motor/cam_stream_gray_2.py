#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
#Motor library
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
import atexit

# Index of wheels
#motor#
#Head##
#1   2#
#     #
#4   3#
#Back##

# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x6f)
#Get_Motor_num
myMotor_1 = mh.getMotor(1)
myMotor_2 = mh.getMotor(2)
myMotor_3 = mh.getMotor(3)
myMotor_4 = mh.getMotor(4)

# motor: recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(2).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(3).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(4).run(Raspi_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

def goStraight():
	print ("Go straight! ")
	#Set Motor Direction
	myMotor_1.run(Raspi_MotorHAT.BACKWARD)
	myMotor_2.run(Raspi_MotorHAT.BACKWARD)
	myMotor_3.run(Raspi_MotorHAT.BACKWARD)
	myMotor_4.run(Raspi_MotorHAT.BACKWARD)
	#Set Motor Speed
	myMotor_1.setSpeed(60)
	myMotor_2.setSpeed(60)
	myMotor_3.setSpeed(60)
	myMotor_4.setSpeed(60)
	time.sleep(0.01)
def turnLeft():
	print ("Turned left!! ")
	#Set Motor Direction
	myMotor_1.run(Raspi_MotorHAT.FORWARD)
	myMotor_2.run(Raspi_MotorHAT.BACKWARD)
	myMotor_3.run(Raspi_MotorHAT.BACKWARD)
	myMotor_4.run(Raspi_MotorHAT.FORWARD)
	#Set Motor Speed
	myMotor_1.setSpeed(60)
	myMotor_2.setSpeed(60)
	myMotor_3.setSpeed(60)
	myMotor_4.setSpeed(60)
	time.sleep(0.01)
def turnRight():
	print ("Turned right! ")
	#Set Motor Direction
	myMotor_1.run(Raspi_MotorHAT.BACKWARD)
	myMotor_2.run(Raspi_MotorHAT.FORWARD)
	myMotor_3.run(Raspi_MotorHAT.FORWARD)
	myMotor_4.run(Raspi_MotorHAT.BACKWARD)
	#Set Motor Speed
	myMotor_1.setSpeed(60)
	myMotor_2.setSpeed(60)
	myMotor_3.setSpeed(60)
	myMotor_4.setSpeed(60)
	time.sleep(0.01)
def Stop():
	print ("Stop ")
	#Set Motor Direction
	myMotor_1.run(Raspi_MotorHAT.BACKWARD)
	myMotor_2.run(Raspi_MotorHAT.FORWARD)
	myMotor_3.run(Raspi_MotorHAT.FORWARD)
	myMotor_4.run(Raspi_MotorHAT.BACKWARD)
	#Set Motor Speed
	myMotor_1.setSpeed(0)
	myMotor_2.setSpeed(0)
	myMotor_3.setSpeed(0)
	myMotor_4.setSpeed(0)
	time.sleep(0.01)
	
class CamStream:
	def __init__(self):
		rospy.init_node("CamStream", anonymous = False)
		rospy.Subscriber("cv_camera/image_raw", Image, self.imageCallback)
		self.pub = rospy.Publisher('CamStream/image', Image, queue_size = 1)
		self.bridge = CvBridge()
	
	def imageCallback(self, image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8") 			# Change to openCV Format
			# resize
			cv_image = cv2.resize(cv_image, (180,120), cv2.INTER_LINEAR)
			# Gaussian Blur
			cv_image_blur = cv2.GaussianBlur(cv_image, (7,7), 0)				
			
			# Color to HSV
			cv_image_hsv = cv2.cvtColor(cv_image_blur, cv2.COLOR_BGR2HSV) 
			# Extract Target Color: Red
			cv_image_extract_red = cv2.inRange(	cv_image_hsv, 
												np.array([ 0, 0, 0]), 
												np.array([ 8, 255, 255]))
			

			#morphology
			morphology = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
			cv_image_extract_red = cv2.erode(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.dilate(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.dilate(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.dilate(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.erode(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.erode(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.erode(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.erode(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.dilate(cv_image_extract_red, morphology)
			cv_image_extract_red = cv2.dilate(cv_image_extract_red, morphology)
			
			#Hough Circle
			circles = cv2.HoughCircles(	cv_image_extract_red, 
										cv2.HOUGH_GRADIENT, 1, 
										minDist=40, 
										param1=70, 
										param2=3, 
										minRadius=3, 
										maxRadius=60)
		
			radius_list = []
			if circles is not None:
				for i in circles[0,:]:
					#找出最大圓且圓心是白點
					if cv_image_extract_red[int(i[1]), int(i[0])] == 255:
						radius_list.append(i[2])
					else:
						radius_list.append(0)
				max_radius_id = np.argmax(radius_list)	
				
				#畫圓
				cv2.circle(	cv_image, 
							(circles[0][max_radius_id][0], circles[0][max_radius_id][1]), 
							circles[0][max_radius_id][2], 
							(0, 255, 0), 2)
				#畫圓心
				cv2.circle(	cv_image, 
							(circles[0][max_radius_id][0], circles[0][max_radius_id][1]), 
							2, (255, 0, 255), 2)
				
				#車子移動
				if circles[0][max_radius_id][2] < 25 :	#如果圓很小
					if abs(circles[0][max_radius_id][0] - 80) > 45:	#如果圓偏離中間
						if circles[0][max_radius_id][0] < 80 :	#圓偏左?
							turnLeft()
							print('left')
						else:
							turnRight()
							print('right')
					else:
						goStraight()
						print('straight')
				else:
					Stop()
			
			else :	#如果沒有找到圓的話,車子持續原地旋轉
				print('HoughCircle: empty')
				turnLeft()
			#resize
			cv_image = cv2.resize(cv_image, (360,240), cv2.INTER_LINEAR)
			image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8") # Change to ROS Forma
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
