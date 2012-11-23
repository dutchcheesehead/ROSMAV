#!/usr/bin/env python
#
# Uses AR tags to create, propagate, and store a collection of
# location estimates.
import roslib ; roslib.load_manifest('ar_filter')
import rospy
from perfesser.srv import Guess
from perfesser.msg import Announce
from ar_recog.msg import Tag, Tags
from nav_msgs.msg import Odometry
import math
from ar_server import *
from ar_guesser import *
from ar_filter.msg import Lmarks

rospy.init_node('ar_filter')

filename = rospy.get_param('/guesser/position/ar_filter/file_name',
                           "/home/obot/ar_tag_locations.txt")
npoints = rospy.get_param('/guesser/position/npoints', 1000)
dError = rospy.get_param('/guesser/position/ar_filter/d_error', 0.2)
angError = rospy.get_param('/guesser/position/ar_filter/ang_error', 0.2)
idError = rospy.get_param('/guesser/position/ar_filter/id_error', 0.95)
motionError = rospy.get_param('/guesser/position/ar_filter/motion_error', 3.0)

annc = rospy.Publisher('guesser/position/guessers', Announce)
lpub = rospy.Publisher('guesser/position/landmarks', Lmarks)
ars = ARServer('ar_filter', npoints, (0.0, 0.0, 2*math.pi),
               dError, angError, idError, motionError,
               announce=annc, locfile=filename, landmarks=lpub)

rospy.Service('ar_filter', Guess, ars.answer_server)
rospy.Subscriber('tags', Tags, ars.arGuesser.handle_image)
rospy.Subscriber('odom', Odometry, ars.arGuesser.handle_odom)
rospy.Subscriber('guesser/position/landmarks', Lmarks, ars.arGuesser.handle_lm)

rospy.spin()
