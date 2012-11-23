#!/usr/bin/python
import roslib; roslib.load_manifest('discreteMove_0_0_1')
import rospy
from discreteMove_0_0_1.srv import *
from irobot import Create
from threading import Lock
from time import sleep

def collision():
	global create, norms, threshold
	sensors = [create.cliffLeftSignal, create.cliffFrontLeftSignal, create.cliffFrontRightSignal, create.cliffRightSignal]
	# the first paren after the colon should eventually be an abs, we removed it so that the dirty (formerly taped spots) could be 
	# properly ignored        h <-- abs under the h
	if (len(filter(lambda x : (float(x[0])-float(x[1]))/float(x[1]) > threshold,zip(sensors,norms))) or create.bumpLeft or create.bumpRight):
		return True
	else:
		return  False

def act(action):
	global bump, rate, lock, speed, turn, create, distance, angle, norms, threshold

	if (not lock.acquire(0)):
		return

	move = action.action

	if (move == 2):
		create.clear()
		while(create.distance != 0):
			sleep(rate)
		create.tank(speed,speed)
		while(create.distance < distance):
			#print map(lambda x: abs(float(x[0])-float(x[1]))/float(x[0]),zip(sensors,norms))
			if (collision()):
				create.tank(-speed,-speed)
				while(create.distance > 0):
					sleep(rate)
				break
			sleep(rate)

	elif (move == 1 or move == 3):
		create.clear()
		while(create.angle != 0):
			sleep(rate)

		toPos = 1
		if (move == 1):
			toPos = -toPos
		create.turn(turn*toPos)

		if (not collision()):
			while(create.angle*toPos*-1 < angle):
				if (collision()):
					break
				sleep(rate)
		else:
			while(collision()):
				sleep(rate)

	create.brake()

	sleep(rate*10)

	lock.release()

	return ActResponse(True)

if __name__ == '__main__':
	global bump, rate, lock, speed, turn, create, distance, angle, norms, threshold
	bump = False
	hz = 30
	rate = 1. / hz
	speed = 200
	turn = 50
	distance = 85
	threshold = .5
	angle = 18
	lock = Lock()

	rospy.init_node('discreteController')

	port = rospy.get_param('/brown/irobot_create_2_1/port', "/dev/ttyUSB0")
	print(port)
	create = Create(port)
	service = rospy.Service('act', Act, act)

	try:
		sleep(1)
		create.start()
		sleep(1)
		norms = [create.cliffLeftSignal, create.cliffFrontLeftSignal, create.cliffFrontRightSignal, create.cliffRightSignal]
		print "\nrunning...\n"
		rospy.spin()
	except:
		pass
	finally:
		create.stop()
		print "\nstopping..."
