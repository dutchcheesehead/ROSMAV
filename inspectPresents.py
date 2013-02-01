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
import heatmap
import numpy as np

import cv
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
hm_pub = rospy.Publisher("/heatmap_image", Image)
hm_enc_pub = rospy.Publisher("/heatmap_image/encoding", String)

action = rospy.Publisher("cmd_vel", Twist)

rospy.init_node('v', anonymous=True)

WIDTH = 320
HEIGHT = 240
MIDPOINTX = 160

directionz = 1

C_LIGHT = (253, 255, 253)
C_TARGET = (0,  182,  234)

target_heatmap = np.zeros(shape=(240/heatmap.downsize_factor, 320/heatmap.downsize_factor), dtype=np.int8)
indexes = np.array(range(320/heatmap.downsize_factor))


#Specifies whether a given color is red by looking at the RGB values
def isRed(c):
	return c[0] > 150
#Specifies whether a given color is green by looking at the RGB values
def isGreen(c):
	return c[1] > 50 and c[1] < 150 and c[0] < 50 and c[2] < 50 and c[2] < 150
#Specifies whether a given color is blue by looking at the RGB values
def isBlue(c):
	return c[0] < 180 and c[1] > 150
#Specifies whether a given color is yellow by looking at the RGB values
def isYellow(c):
	return c[0] > 150 and c[1] > 150 and c[2] < 200

def finished(c): return False

currentTarget = isRed
nextTarget = {isRed: isBlue, isBlue: isRed}

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
		#action.publish(twist)
		print "n/a"
	print currentTarget.__name__
	try:
		print getat(img, max(data.blobs, key=lambda x: x.area))
	except:
		pass
	targets = [x for x in data.blobs if currentTarget(getat(img, x))]
	heatmap.cooldown(target_heatmap)
	heatmap.draw(target_heatmap, targets)
	hm2 = 1 * target_heatmap
	hm2[hm2 < 80] = 0
	
	f = cv.fromarray(hm2)
	#print f, dir(f), f.step, f.channels, f.cols, f.rows, f.width, f.height
	hm_pub.publish(bridge.cv_to_imgmsg(f))
	hm_enc_pub.publish("16SC1")
	if targets:
		# When we found targets we start to inspect them/fly towards them
		target = max(targets, key=lambda x: x.area)
		twist = Twist()
		print target.area
		# If the target found is very large we assume that the target is found. Otherwise we fly towards the target
		if target.area > 2500:
			target_heatmap[:] = 0
			# If we have a new target after this one, take the next target. Otherwise we start landing
			if currentTarget in nextTarget:
				currentTarget = nextTarget[currentTarget]
			else:
				# Start landing
				twist.linear.x = -.1
				action.publish(twist)
				l = rospy.Publisher("/ardrone/land", Empty)
				l.publish(Empty())
				print "found"
				currentTarget = finished
		else:
			# Fly forward
			twist.linear.x = .1
			if target.x < MIDPOINTX:
				# If the target is to the left, fly to the left
				twist.angular.z = .1
				print "left (old)"
			else:
				# If the target is to the right, fly to the right
				twist.angular.z = -.1
				print "right (old)"
	twist = Twist()
	# When a dot in the activation matrix is high enough we start to fly towards this place
	if hm2.max() > 16:
		weights = np.apply_along_axis(np.sum, 0, hm2)
		loc = np.average(indexes, weights=weights)/len(indexes) # When the target if farther to the left or right, we turn faster
		# Print what direction we are going to fly towards
		if loc < .5:
			print "left (new)"
		else:
			print "right (new)"
		twist.linear.x = .1
		twist.angular.z = .5 - loc
	action.publish(twist)

img = None

# The img_get function is called every time the topic ardrone/img_raw fires. It saves the captured image to a global variable
def img_get(data):
	global img
	img = data

rospy.Subscriber("/blobs", Blobs, c)
rospy.Subscriber("/ardrone/image_raw", Image, img_get)

try:
	rospy.spin()
except KeyboardInterrupt:
	print "Shutting down"

