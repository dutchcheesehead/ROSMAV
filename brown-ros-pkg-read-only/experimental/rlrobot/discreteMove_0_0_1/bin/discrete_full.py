#!/usr/bin/python
import roslib; roslib.load_manifest('discreteMove_0_0_1')
import rospy
from discreteMove_0_0_1.srv import *
from irobot import Create
from threading import Lock
from time import sleep
from math import sin,cos,pi
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from datetime import datetime
from irobot_create_2_1.msg import SensorPacket

class discreteCreateDriver:
   def __init__(self):
       print("init")
       port = rospy.get_param('/brown/irobot_create_2_1/port', "/dev/rfcomm0")
       self.create=Create(port)

       self.bump=False
       self.hz=30
       self.rate = 1. / self.hz
       self.speed = 200
       self.turn = 50
       self.distance = 85
       self.threshold = .5
       self.angle = 18
       self.lock = Lock()
       self.packetPub = rospy.Publisher('sensorPacket', SensorPacket)

       self.odomPub=rospy.Publisher('odom', Odometry)
       self.odomBroadcaster=TransformBroadcaster()
       self.then=datetime.now()
       self.x=0
       self.y=0
       self.th=0
       self.fields = ['wheeldropCaster','wheeldropLeft','wheeldropRight','bumpLeft','bumpRight','wall','cliffLeft','cliffFronLeft','cliffFrontRight','cliffRight','virtualWall','infraredByte','advance','play','distance','angle','chargingState','voltage','current','batteryTemperature','batteryCharge','batteryCapacity','wallSignal','cliffLeftSignal','cliffFrontLeftSignal','cliffFrontRightSignal','cliffRightSignal','homeBase','internalCharger','songNumber','songPlaying']
       
       self.create.update = self.sense

   def start(self):
       self.create.start()
       self.then=datetime.now()
       self.norms = [self.create.cliffLeftSignal, self.create.cliffFrontLeftSignal, self.create.cliffFrontRightSignal, self.create.cliffRightSignal]

   def stop(self):
       self.create.stop()
       
   def sense(self):
       print("sensing\n")	   
       now = datetime.now()
       elapsed = now - self.then
       self.then = now
       elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.
       d = self.create.d_distance / 1000.
       th = self.create.d_angle*pi/180
       dx = d / elapsed
       dth = th / elapsed

       if (d != 0):
	       x = cos(th)*d
	       y = -sin(th)*d
	       self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
	       self.y = self.y + (sin(self.th)*x + cos(self.th)*y)

       if (th != 0):
	       self.th = self.th + th

       quaternion = Quaternion()
       quaternion.x = 0.0 
       quaternion.y = 0.0
       quaternion.z = sin(self.th/2)
       quaternion.w = cos(self.th/2)
       
       self.odomBroadcaster.sendTransform(
	       (self.x, self.y, 0), 
	       (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
	       rospy.Time.now(),
	       "base_link",
	       "odom"
	       )

       odom = Odometry()
       odom.header.frame_id = "odom"
       odom.pose.pose.position.x = self.x
       odom.pose.pose.position.y = self.y
       odom.pose.pose.position.z = 0
       odom.pose.pose.orientation = quaternion
       
       odom.child_frame_id = "base_link"
       odom.twist.twist.linear.x = dx
       odom.twist.twist.linear.y = 0
       odom.twist.twist.angular.z = dth
       
       self.odomPub.publish(odom)
       packet = SensorPacket()
       for field in self.fields:
	       packet.__setattr__(field,self.create.__getattr__(field))
       self.packetPub.publish(packet)


   def collision(self):
       #global create, norms, threshold
       sensors = [self.create.cliffLeftSignal, self.create.cliffFrontLeftSignal, self.create.cliffFrontRightSignal, self.create.cliffRightSignal]
       # the first paren after the colon should eventually be an abs, we removed it so that the dirty (formerly taped spots) could be 
       # properly ignored        h <-- abs under the h
       if (len(filter(lambda x : (float(x[0])-float(x[1]))/float(x[1]) > self.threshold,zip(sensors,self.norms))) or self.create.bumpLeft or self.create.bumpRight):
	       return True
       else:
	       return  False

   def act(self, action):
       #global bump, rate, lock, speed, turn, create, distance, angle, norms, threshold
	   
	   if (not self.lock.acquire(0)):
		   return
	   
	   move = action.action
	   
	   if (move == 1):
		   self.create.clear()
		   while(self.create.distance != 0):
			   sleep(self.rate)
		   self.create.tank(self.speed,self.speed)
		   while(self.create.distance < self.distance):
			#print map(lambda x: abs(float(x[0])-float(x[1]))/float(x[0]),zip(sensors,norms))
			   if (self.collision()):
				   self.create.tank(-self.speed,-self.speed)
				   while(self.create.distance > 0):
					   sleep(self.rate)
				   break
			   sleep(self.rate)

	   elif (move == 0 or move == 2):
		   self.create.clear()
		   while(self.create.angle != 0):
			   sleep(self.rate)

		   toPos = 1
		   if (move == 0):
			   toPos = -toPos
		   self.create.turn(self.turn*toPos)

		   if (not self.collision()):
			   while(self.create.angle*toPos*-1 < self.angle):
				   if (self.collision()):
					   break
				   sleep(self.rate)
		   else:
			   while(self.collision()):
				   sleep(self.rate)

	   self.create.brake()

	   sleep(self.rate*10)

	   self.lock.release()
	
	   return ActResponse(True)

if __name__ == '__main__':
	rospy.init_node('discreteController')
	
	driver = discreteCreateDriver()


	service = rospy.Service('act', Act, driver.act)

#	try:
	sleep(1)
	driver.start()
	sleep(1)
	print "\nrunning...\n"
	rospy.spin()
	#except:
	#	print("Going to Pass\n")
	#	pass
	#finally:
	print "\nstopping..."
	driver.stop()

