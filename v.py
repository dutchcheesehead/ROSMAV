#!/usr/bin/env python
import roslib
import time
roslib.load_manifest('lijnvolger')
import sys
import rospy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cmvision.msg import Blobs, Blob
from getat import getat
import dominant

action = rospy.Publisher("cmd_vel", Twist)

rospy.init_node('v', anonymous=True)

WIDTH = 320
HEIGHT = 240
MIDPOINTX = 160

directionz = 1

C_LIGHT = (253, 255, 253)
C_TARGET = (0,  182,  234)

def isRed(c):
	return c[0] > 100 and c[0] < 255 and c[1] < 250 and c[2] < 250
def isGreen(c):
	return c[1] > 50 and c[1] < 150 and c[0] < 50 and c[2] < 50 and c[2] < 150
def isBlue(c):
	return c[2] > 150 and c[0] < 100 and c[1] > 150
def isYellow(c):
	return c[0] > 150 and c[1] > 150 and c[2] < 200
def isTarget(c):
	return c[0] <= 100
def finished(c): return False

currentTarget = isRed#isBlue#isTarget
nextTarget = {isRed: isBlue, isBlue: isRed}#{isBlue: isRed, isRed: isGreen, isGreen: isBlue}

def c(data):
	global directionz, currentTarget
	if img is None:
		return
	lights = [x for x in data.blobs if getat(img, x)[0] > 100]
	if lights and False:
		naar = min(lights, key=lambda x: x.y)
		twist = Twist()
		twist.linear.x = .1
		if naar.x < 50:
			twist.angular.z = .7
			directionz = 1
			print "hard left"
		elif naar.y > WIDTH - 50:
			twist.angular.z = -.7
			directionz = -1
			print "hard right"
		elif naar.x < MIDPOINTX - 20:
			twist.angular.z = .3
			directionz = 1
			print "left"
		elif naar.x > MIDPOINTX + 20:
			twist.angular.z = -.3
			directionz = -1
			print "right"
		else:
			#twist.linear.x = .2
			print "straight on"
		action.publish(twist)
	else:
		twist = Twist()
		twist.angular.z = directionz
		action.publish(twist)
		print "n/a"
	print currentTarget.__name__
	try:
		print getat(img, max(data.blobs, key=lambda x: x.area))
	except:
		pass
	targets = [x for x in data.blobs if currentTarget(getat(img, x))]
	if targets:
		target = max(targets, key=lambda x: x.area)
		twist = Twist()
		if target.area > 1000:
			#found!
			if currentTarget in nextTarget:
				currentTarget = nextTarget[currentTarget]
			else:
				twist.linear.x = -.1
				action.publish(twist)
				l = rospy.Publisher("/ardrone/land", Empty)
				l.publish(Empty())
				print "found"
				currentTarget = finished
		else:
			twist.linear.x = .1
			if target.x < MIDPOINTX:
				twist.angular.z = .5
			else:
				twist.angular.z = -.5
			action.publish(twist)
			print "closer"

img = None

def img_get(data):
	#print "#################################started img_get"
	global img
	img = data
	#t = time.time()
	#print dominant.colors(img, 1)
	#print time.time() - t

rospy.Subscriber("/blobs", Blobs, c)
rospy.Subscriber("/ardrone/image_raw", Image, img_get)

try:
	rospy.spin()
except KeyboardInterrupt:
	print "Shutting down"

