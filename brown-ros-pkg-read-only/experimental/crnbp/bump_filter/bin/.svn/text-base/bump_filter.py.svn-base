#!/usr/bin/env python
#
# Uses bumps to create, propagate, and store a collection of
# location estimates.
import roslib ; roslib.load_manifest('bump_filter')
import rospy
from perfesser.srv import Guess
from perfesser.msg import Announce
from nav_msgs.msg import OccupancyGrid
import math
#from bump_guesser import *
from bump_server import *

rospy.init_node('bump_filter')

npoints = rospy.get_param('/guesser/position/npoints', 1000)

annc = rospy.Publisher('/guesser/position/guessers', Announce)
ptb = rospy.Publisher('/step_nav/perturb', Perturb)
bs = BumpServer('bump_filter', npoints, (10,10), 0.95, announce=annc,
                perturb=ptb)

rospy.Service('bump_filter', Guess, bs.answer_server)

# This only gets called once.
rospy.Subscriber('map', OccupancyGrid, bs.addMap)

rospy.Subscriber('sensorPacket', SensorPacket, bs.checkSensorPacket)

rospy.spin()
