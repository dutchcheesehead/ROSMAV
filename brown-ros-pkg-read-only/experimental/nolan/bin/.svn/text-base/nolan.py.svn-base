#!/usr/bin/env python
import roslib; roslib.load_manifest('nolan')
import rospy

from ar_recog.msg import Tag, Tags
from geometry_msgs.msg import Twist

from time import time

def handleTags(msg):
	global pub
	global lastDir
	global lastSeen
	
	width = msg.image_width
	height = msg.image_height

	biggest = Tag()
	for tag in msg.tags:
		if (tag.diameter > biggest.diameter):
			biggest = tag

	if biggest.diameter == 0:
		twist = Twist()
		twist.linear.x = 0
		if (time() - lastSeen > .5):
			twist.angular.z = .5*lastDir
		pub.publish(twist)
		return
	
	lastSeen = time()

	cx = 0; cy = 0
	for i in [0,2,4,6]:
		cx = cx + biggest.cwCorners[i]
		cy = cy + biggest.cwCorners[i+1]
	cx = cx / 4. / width
	cy = cy / 4. / height

	twist = Twist()
	if (biggest.distance - 500 > 8):
		twist.linear.x = ((biggest.distance - 500.) / 500.) * .25
	if (twist.linear.x < 0):
		twist.linear.x = 0
	twist.angular.z = (-(cx - .5)/.5)
	if (twist.angular.z < 0):
		lastDir = -1
	else:
		lastDir = 1
	pub.publish(twist)

if __name__ == "__main__":
	global pub
	global lastDir 
	global lastSeen

	lastSeen = 0
	lastDir = -1

	rospy.init_node('nolan', anonymous=True)
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.Subscriber("tags", Tags, handleTags)
	rospy.spin()
