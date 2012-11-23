#!/usr/bin/env python
#
# Uses map data to edit a collection of location estimates in
# cooperation with the perfesser guesser.
#
# We'll do this at first with a single big map, and later allow operation 
# on the smaller relevant portion of the map.
import roslib ; roslib.load_manifest('map_filter')
import rospy
from perfesser.srv import Guess
from perfesser.msg import Announce
import math
from map_guesser import *
from map_server import *

rospy.init_node('map_filter')

npoints = rospy.get_param('/guesser/position/npoints', 1000)

annc = rospy.Publisher('/guesser/position/guessers', Announce)
ms = MapServer('map_filter', npoints, (0.0, 0.0, 2*math.pi), 
                0.1, 0.1, 0.1, announce=annc)

mapfile = rospy.get_param('guesser/position/map_filter/map_file', '/home/obot/map.txt')

ms.addMap(mapfile)

sleep(5.0)

rospy.Service('map_filter', Guess, ms.answer_server)

rospy.spin()


