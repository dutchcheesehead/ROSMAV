#!/usr/bin/env python
import roslib; roslib.load_manifest('headtrack')
import rospy
from NAOcontrol.msg import Arm
from NAOcontrol.msg import ArmAngles
from std_msgs.msg import String

from sys import argv,exit
from math import *
import numpy
from numpy import matrix
import threading
from threading import Thread

	
if __name__ == '__main__':
	angleStep = .1
	lock = threading.Lock()
	rospy.init_node('naoIk', anonymous=False)
	currArmAngles = ArmAngles();
	TO_RAD = 3.14/180.0 
	currArmAngles.armAngles = [80.0 * TO_RAD, -40.0 * TO_RAD, 80.0 * TO_RAD,  80.0 * TO_RAD, 0.0 * TO_RAD, 1.0]
	currArmAngles.duration = 2.0;
	pub = rospy.Publisher('RArmAngles', ArmAngles)
	pub2 = rospy.Publisher('RArmPose',Arm)
	minAngles = [-2.094, -1.658, -2.094, 0.0, -1.832, 0.0]
	maxAngles = [2.094, 0.0, 2.094, 1.658, 2.617, 1.0]
	rospy.sleep(2.0)
	pub.publish(currArmAngles)
	rospy.sleep(2.0)
	currArmAngles.duration = .1;
	t3 = matrix([[0],[0],[1]])
	t2 = matrix([[0.11],[0],[0]])
	t1 = matrix([[0.09],[0],[0]])
	a = currArmAngles.armAngles;
	r1 = matrix([[1,0,0],[0,cos(a[1]),-sin(a[1])],[0,sin(a[1]),cos(a[1])]])
	r2 = matrix([[cos(a[0]),0,sin(a[0])],[0,1,0],[-sin(a[0]),0,cos(a[0])]])
	r3 = matrix([[1,0,0],[0,cos(a[2]),-sin(a[2])],[0,sin(a[2]),cos(a[2])]])
	r4 = matrix([[cos(a[3]),-sin(a[3]),0],[sin(a[3]),cos(a[3]),0],[0,0,1]])
	tmp = r3*r4*t2;
	currPose = r1*r2*(t1+tmp)
	def do_ik(arm):
		global currPose
		if(not lock.acquire(0)):
			return
		target = arm.armPose
		if(arm.hand < .3):
			currArmAngles.armAngles[5] = 0.0
			print "closing hand"
		elif(arm.hand > .7):
			arm.hand = 1.0
		#print "Current Pose"
		#print currPose
		tarMat = matrix([[target[0]],[target[1]],[target[2]]])
		#print tarMat
		score = [0.0]*8
		for i in xrange(4):
			a[i] = a[i]+angleStep
			r1 = matrix([[1,0,0],[0,cos(a[1]),-sin(a[1])],[0,sin(a[1]),cos(a[1])]])
			r2 = matrix([[cos(a[0]),0,sin(a[0])],[0,1,0],[-sin(a[0]),0,cos(a[0])]])
			r3 = matrix([[1,0,0],[0,cos(a[2]),-sin(a[2])],[0,sin(a[2]),cos(a[2])]])
			r4 = matrix([[cos(a[3]),-sin(a[3]),0],[sin(a[3]),cos(a[3]),0],[0,0,1]])
			tmp = r3*r4*t2;
			newPose = r1*r2*(t1+tmp)
			score[i] = sum(numpy.square(tarMat -newPose))
			a[i] = a[i] -angleStep
		for i in xrange(4):
			a[i] = a[i]-angleStep
			r1 = matrix([[1,0,0],[0,cos(a[1]),-sin(a[1])],[0,sin(a[1]),cos(a[1])]])
			r2 = matrix([[cos(a[0]),0,sin(a[0])],[0,1,0],[-sin(a[0]),0,cos(a[0])]])
			r3 = matrix([[1,0,0],[0,cos(a[2]),-sin(a[2])],[0,sin(a[2]),cos(a[2])]])
			r4 = matrix([[cos(a[3]),-sin(a[3]),0],[sin(a[3]),cos(a[3]),0],[0,0,1]])
			tmp = r3*r4*t2;
			newPose = r1*r2*(t1+tmp)
			score[i+4] = sum(numpy.square(tarMat -newPose))
			a[i] = a[i] +angleStep
		mi = -1
		mn = sum(numpy.square(tarMat - currPose))
		#print "Current error"
		#print mn
		#print "new scores"
		#print score
		for i in xrange(4):
			if (score[i] < mn and a[i]+angleStep < maxAngles[i]):
				mn = score[i]
				mi = i
			pass
		for i in xrange(4):
			if (score[i+4] < mn and a[i]-angleStep > minAngles[i]):
				mn = score[i+4]
				mi = i+4
			pass
		#print mi
		if(mi < 0):
			pass
		elif(mi < 4):
			currArmAngles.duration = .1
			a[mi] = a[mi] + angleStep
			currArmAngles.armAngles = a
		else:
			currArmAngles.duration = .1
			a[mi-4] = a[mi-4] - angleStep
			currArmAngles.armAngles = a
		r1 = matrix([[1,0,0],[0,cos(a[1]),-sin(a[1])],[0,sin(a[1]),cos(a[1])]])
		r2 = matrix([[cos(a[0]),0,sin(a[0])],[0,1,0],[-sin(a[0]),0,cos(a[0])]])
		r3 = matrix([[1,0,0],[0,cos(a[2]),-sin(a[2])],[0,sin(a[2]),cos(a[2])]])
		r4 = matrix([[cos(a[3]),-sin(a[3]),0],[sin(a[3]),cos(a[3]),0],[0,0,1]])
		tmp = r3*r4*t2;
		currPose = r1*r2*(t1+tmp)
		zaxis = r1*r2*r3*r4*t3
		wx = atan2(zaxis[2],sqrt(zaxis[1]*zaxis[1]+zaxis[0]*zaxis[0]))
		if(wx > minAngles[4] and wx < maxAngles[4]):
			currArmAngles.armAngles[4] = wx
		arm2 = Arm()
		for i in xrange(3):
			arm2.armPose[i] = currPose[i]
		arm2.hand = currArmAngles.armAngles[5]
		print arm2.armPose
		pub2.publish(arm2)
		if( mi > 0):
			pub.publish(currArmAngles)
			rospy.sleep(.1)
			
		lock.release()


	rospy.Subscriber("RArmIK", Arm, do_ik)
	rospy.spin()

