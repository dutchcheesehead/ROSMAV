#!/usr/bin/env python
#
# This ROS node simply listens to the position guesses and the sensor
# packets.  When a bump is detected, it estimates the location of the
# obstacle based on the latest position guess, and broadcasts the bump
# estimate on the bump channel.
import roslib ; roslib.load_manifest('bump_mapper')
import rospy
from irobot_create_2_1a.msg import SensorPacket
from geometry_msgs.msg import Twist
from perfesser.msg import Belief, Pt
import math
from bump_mapper import *

rospy.init_node('bump_mapper')

bpub = rospy.Publisher('/guesser/bumps/obstacles', Belief) 

bm = BumpMapper(pub=bpub)

rospy.Subscriber('cmd_vel', Twist, bm.trackVelocity)
rospy.Subscriber('sensorPacket', SensorPacket, bm.checkBump)
rospy.Subscriber('/guesser/position/guess', Belief, bm.trackPosition)

rospy.spin()
