#!/usr/bin/python
import os
import sys

path = os.environ.get("AL_PYTHON_PATH")
sys.path.append(path)
#sys.path.append("../pythonModules")

import math
import roslib
roslib.load_manifest('NAOcontrol')
roslib.load_manifest('std_msgs')
import rospy

from NAOcontrol.msg import HeadF
from NAOcontrol.msg import Head
from NAOcontrol.msg import Walk
from NAOcontrol.msg import Arm
from NAOcontrol.msg import ArmAngles
from std_msgs.msg import String
import naoqi
from naoqi import ALProxy

import threading
from threading import Thread

import time

class walkIt(Thread):
	def __init__(self):
		Thread.__init__(self)
	def run(self):
		global quit
		try:
			while(not quit):
				motion3.walk()
				rospy.sleep(0.05)
		except:
			pass


def handleSpeech(data):
	tts.say(data.data)

def handleHead(data):
	global headLock

	if (not headLock.acquire(0)):
		return

	x = data.x
	y = data.y
	xAngle = 1.4*(x/256.)-.7
	yAngle = 1.4*(y/256.)-.7

	try:
		motion.setAngle("HeadYaw", -xAngle)
		motion.setAngle("HeadPitch", yAngle)
	except:
		print "problem"
	finally:
		headLock.release()

def handleHeadF(data):
        global headLock

        if (not headLock.acquire(0)):
                return

        xAngle = data.x
        yAngle = data.y

        try:
                motion.setAngle("HeadYaw", xAngle)
                motion.setAngle("HeadPitch", yAngle)
        except:
                print "problem"
        finally:
                headLock.release()
                
def handleArmAngles(data):
	global armLock
	if (not armLock.acquire(0)):
		return
		
	angles = data.armAngles
	angles2 = [0]*6
	for i in xrange(len(angles)):
		angles2[i] = angles[i]
	print angles2
	try:
		motion4.gotoChainAngles('RArm',angles2,data.duration,motion.INTERPOLATION_SMOOTH)
	except:
		print "ArmAnglesProblem"
	finally:
		armLock.release()

def handleArm(data):
	global armLock

	if (not armLock.acquire(0)):
		return
		
	inPose = motion4.getPosition("RArm",0)
	target = data.armPose
	print "target: ",target
	outPose = inPose
	s = 0
	for i in xrange(3):
		s = s + abs(target[i] - inPose[i])
	print "curr pose: ",inPose
	#print s
	for i in xrange(3):
		if(s > .08):
			outPose[i] = inPose[i] + (target[i]-inPose[i])*.08/s
		else:
			outPose[i] = target[i]
		pass#outPose[i+3] = target[i+3]
	try:
		if(s > .01):
			print "moving to: ",outPose
			motion4.gotoPosition('RArm',1,outPose,6, .2,0)
			currPose = motion4.getPosition("RArm",0)
			print "resulting pose: ",currPose
		if (data.hand == 0):
			print "Trying to close hand"
			motion4.closeHand('RHand')
		elif(data.hand == 1):
			print "Trying to open hand"
			motion4.openHand('RHand')
	except:
		print "problem"
	finally:
		armLock.release()
		print
		
def clear():
	#this function must occur in a locked context
	motion2.clearFootsteps()
	while(motion2.walkIsActive()):
		rospy.sleep(0.05)

def handleMotion(data):
	global lastMove
	global lock

	if (not lock.acquire(0)):
		return

	steps = 75

	forward = False
	left = False
	right = False

	if (data.walk == 2 or data.walk == 3):
		left = True
	if (data.walk == 4 or data.walk == 5):
		right = True
	if (data.walk == 1 or data.walk == 3 or data.walk == 5):
		forward = True
	

	if (left and not forward):
		if (lastMove != "leftForward"):
			clear()
			lastMove = "leftForward"
		motion2.addTurn(.6,steps)
	elif (right and not forward):
		if (lastMove != "rightForward"):
			clear()
			lastMove = "rightForward"
		motion2.addTurn(-.6,steps)
	elif (left):
		if (lastMove != "left"):
			clear()
			lastMove = "left"
		motion2.addWalkArc(1.57,.3,steps)
	elif (right):
		if (lastMove != "right"):
			clear()
			lastMove = "right"
		motion2.addWalkArc(-1.57,.3,steps)
	elif (forward):
		if (lastMove != "forward"):
			clear()
			lastMove = "forward"
		motion2.addWalkStraight(.1,steps)
	else:
		if (lastMove != "stop"):
			clear()

	lock.release()

def motListener():
	rospy.Subscriber("motion", Walk, handleMotion, queue_size=1)

def strListener():
	rospy.Subscriber("speech", String, handleSpeech)
	#rospy.Subscriber("speech", String, handleSpeech, queue_size=1)

def hdListener():
	rospy.Subscriber("head", Head, handleHead, queue_size=1)
	rospy.Subscriber("headF", HeadF, handleHeadF, queue_size=1)

def armListener():
	rospy.Subscriber("RArm", Arm, handleArm, queue_size=1)
	rospy.Subscriber("RArmAngles",ArmAngles,handleArmAngles,queue_size=1)

if __name__ == '__main__':
	#tts = ALProxy("ALTextToSpeech","127.0.0.1",9559)
	#tts.setSystemVolume(80)
	motion = ALProxy("ALMotion","127.0.0.1",9559)
	motion2 = ALProxy("ALMotion","127.0.0.1",9559)
	motion3 = ALProxy("ALMotion","127.0.0.1",9559)
	motion4 = ALProxy("ALMotion","127.0.0.1",9559)
	#watchdog = ALProxy("ALSentinel","127.0.0.1",9559)

	motion2.killAll()
	motion2.setWalkArmsEnable(False)
	motion.setWalkArmsEnable(False)

	try:
		from walkConfig import walkConfig as wc
		motion2.setWalkConfig(wc[0], wc[1], wc[2], wc[3], wc[4], wc[5]) 
	except:
		pass

	lastMove = "stop"
	lock = threading.Lock()
	headLock = threading.Lock()
	armLock = threading.Lock()

	rospy.init_node('listener', anonymous=True)

	hdListener()
	strListener()
	motListener()
	armListener()

	walker = walkIt()

	motion.gotoBodyStiffness(1.0,1.0,1)
	TO_RAD = 3.14/180.0 
	aint = [80.0 * TO_RAD, -20.0 * TO_RAD, 80.0 * TO_RAD,  80.0 * TO_RAD, 0.0, 0.0]
	armLock.acquire(1)
	motion.gotoChainAngles('RArm',aint,0.5,motion.INTERPOLATION_SMOOTH)
	armLock.release()
	quit = False
	walker.start()

	try:
		print "teleop service running"
		rospy.spin()
	except:
		print "stopping teleop service"
	finally:
		quit = True
		#watchdog.crouchAndRemoveStiffness(True)
		time.sleep(1)
		print "stopping"

