#!/usr/bin/python
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x6f)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(2).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(3).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(4).run(Raspi_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

################################# DC motor test!

myMotor_1 = mh.getMotor(1)
myMotor_2 = mh.getMotor(2)
myMotor_3 = mh.getMotor(3)
myMotor_4 = mh.getMotor(4)

while (True):
	myMotor_1.run(Raspi_MotorHAT.FORWARD)
	myMotor_2.run(Raspi_MotorHAT.FORWARD)
	myMotor_3.run(Raspi_MotorHAT.FORWARD)
	myMotor_4.run(Raspi_MotorHAT.FORWARD)
	
	myMotor_1.setSpeed(60)
	myMotor_2.setSpeed(60)
	myMotor_3.setSpeed(60)
	myMotor_4.setSpeed(60)
	print('1')
	'''
	print ("Forward! ")
	myMotor.run(Raspi_MotorHAT.FORWARD)

	print ("\tSpeed up...")
	for i in range(255):
		myMotor.setSpeed(i)
		time.sleep(0.01)

	print ("\tSlow down...")
	for i in reversed(range(255)):
		myMotor.setSpeed(i)
		time.sleep(0.01)

	print ("Backward! ")
	myMotor.run(Raspi_MotorHAT.BACKWARD)

	print ("\tSpeed up...")
	for i in range(255):
		myMotor.setSpeed(i)
		time.sleep(0.01)

	print ("\tSlow down...")
	for i in reversed(range(255)):
		myMotor.setSpeed(i)
		time.sleep(0.01)

	print ("Release")
	myMotor.run(Raspi_MotorHAT.RELEASE)
	time.sleep(1.0)
	'''
