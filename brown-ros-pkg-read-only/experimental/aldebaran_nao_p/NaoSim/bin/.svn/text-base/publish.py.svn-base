#!/usr/bin/python
import os
import sys

path = os.environ.get("AL_PYTHON_PATH")
sys.path.append(path)
#sys.path.append("../pythonModules")

import math
import roslib
roslib.load_manifest('NAOcontrol')
import rospy

from NAOcontrol.msg import Head
from NAOcontrol.msg import Arm
from NAOcontrol.msg import ArmAngles
import naoqi
from naoqi import ALProxy


import time


def send():
	motion = ALProxy("ALMotion","127.0.0.1",9559)

	rospy.init_node('observations')

	headpub = rospy.Publisher('headLoc', Head)
	armpub = rospy.Publisher('RArmLoc',Arm)
	rarm_angle_pub = rospy.Publisher('RArmAnglesLoc',ArmAngles)
	while not rospy.is_shutdown():
		x = int((motion.getAngle("HeadYaw")+.7)*256/1.4)
		y = int((motion.getAngle("HeadPitch")+.7)*256/1.4)
		head = Head()
		head.x = x
		head.y = y
		headpub.publish(head)
		stiff = motion.getChainStiffnesses("RArm")
		rarm = motion.getChainAngles("RArm")
		armAngles = ArmAngles()
		armAngles.duration = 0.0
		for i in range(len(rarm)):
			armAngles.armAngles[i] = rarm[i]
		if (stiff[0] < .1):
			motion.gotoChainAngles("RArm",rarm,0.2,motion.INTERPOLATION_SMOOTH)
		armPos = motion.getPosition("RArm",0)
		arm = Arm()
		for i in range(len(armPos)):
			arm.armPose[i] = armPos[i]
		arm.hand = motion.getAngle("RHand")
		armpub.publish(arm)
		rarm_angle_pub.publish(armAngles)
		rospy.sleep(.1)
	pass




if __name__ == '__main__':

	try:
		print "Getting observatiosn"
		send()
	except:
		print "stopping teleop service"
	finally:
		quit = True
		time.sleep(5)
		print "stopping"

