#!/usr/bin/env python
import roslib
roslib.load_manifest('lijnvolger')
import sys
import rospy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cmvision.msg import Blobs, Blob

action = rospy.Publisher("cmd_vel", Twist)

rospy.init_node('v', anonymous=True)

WIDTH = 320
HEIGHT = 240
MIDPOINTX = 160

directionz = 1

C_LIGHT = (253, 255, 253)
C_TARGET = (0,  182,  234)

def getat(d):
	x, y = d.x, d.y
	return (ord(img.data[x * 3 + y * img.step]),
		ord(img.data[x * 3 + y * img.step + 1]),
		ord(img.data[x * 3 + y * img.step + 2]))

def c(data):
	global directionz
	#print ';  '.join(str(getat(x)) for x in data.blobs)
	#print ';  '.join(','.join([str(x.red), str(x.green),str(x.blue)]) for x in data.blobs)
	lights = [x for x in data.blobs if getat(x)[0] > 100]
	if lights:
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
	targets = [x for x in data.blobs if getat(x)[0] <= 100]
	if targets:
		target = max(targets, key=lambda x: x.area)
		twist = Twist()
		if target.area > 1000:
			#found!
			twist.linear.x = -.1
			action.publish(twist)
			l = rospy.Publisher("/ardrone/land", Empty)
			l.publish(Empty())
			print "found"
		else:
			twist.linear.x = .1
			if target.x < MIDPOINTX:
				twist.angular.z = .5
			else:
				twist.angular.z = -.5
			action.publish(twist)
			print "closer"

	#print data
	#print dir(data)
img = None

def img_get(data):
	global img
	img = data

rospy.Subscriber("/blobs", Blobs, c)
rospy.Subscriber("/ardrone/image_raw", Image, img_get)

try:
	rospy.spin()
except KeyboardInterrupt:
	print "Shutting down"

