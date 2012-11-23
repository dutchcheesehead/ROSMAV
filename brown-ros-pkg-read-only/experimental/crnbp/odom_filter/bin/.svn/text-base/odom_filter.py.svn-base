#!/usr/bin/env python
#
# Uses error estimates to create, propagate, and store a collection of
# location estimates.  The goal is to deal with an arbitrary collection of
# periodic and non-periodic dimensions, but it currently just converts
# the odometry information into (x,y,theta) coordinates and processes
# in that realm.
#
# See odom_guesser.py for details.
#
import roslib ; roslib.load_manifest('odom_filter')
import rospy
from nav_msgs.msg import Odometry
from perfesser.srv import Guess
from perfesser.msg import Announce
from irobot_create_2_1a.msg import SensorPacket
import math
from odom_guesser import *

rospy.init_node('odom_filter')

npoints = rospy.get_param('/guesser/position/npoints', 1000)
posError = rospy.get_param('/guesser/position/odom_filter/pos_error', 0.03)
thetaError = rospy.get_param('/guesser/position/odom_filter/theta_error', 0.07)
phiError = rospy.get_param('/guesser/position/odom_filter/phi_error', 0.02)

annc = rospy.Publisher('guesser/position/guessers', Announce)

og = OdomGuesser('odom_filter', npoints,
                posError, thetaError, phiError, announce=annc, debug=True)

rospy.Service('odom_filter', Guess, og.updateBelief)
rospy.Subscriber('odom', Odometry, og.updateOdom)
rospy.Subscriber('guesser/bumps/obstacles', SensorPacket, og.trackBumps)

rospy.spin()


