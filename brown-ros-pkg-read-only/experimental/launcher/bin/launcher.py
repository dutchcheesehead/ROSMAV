#!/usr/bin/env python
import roslib; roslib.load_manifest('launcher')
import sys

from launcher.srv import *
import rospy

from os import system

def launch(msg):
	resp = False
	try:
		FILE = open(msg.filename,'r')
		resp = True
		print "launching %s" % (msg.filename,)
		system('roslaunch %s &' % (msg.filename,))
	except:
		e = sys.exc_info()[1]
		print "Problem: %s" % (e,)

	return resp

if __name__ == "__main__":
	rospy.init_node('launcher')
	service = rospy.Service('launch', Launch, launch)
	rospy.spin()
