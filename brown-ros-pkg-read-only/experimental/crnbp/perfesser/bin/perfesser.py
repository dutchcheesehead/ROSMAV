#!/usr/bin/env python
#
# The executable version of the perfesser.  Please see perfesser_guesser.py 
# for details about the perfesser's operation overall. 
#
import roslib ; roslib.load_manifest('perfesser')
import rospy
from perfesser_guesser import *
from perfesser_server import *
from perfesser.msg import Belief, Announce
from perfesser.srv import Guess, GuessRequest, GuessResponse
import math

service_stack = []
services = {}



if __name__ == '__main__':
    rospy.init_node('perfesser')

    pub = rospy.Publisher('guesser/position/guess', Belief)

    npoints = rospy.get_param('/guesser/position/npoints', 1000)
    wait_time = rospy.get_param('/guesser/position/wait_time', 0.1)

    guesser = Guesser('perfesser', (0.0,0.0,2*math.pi), 'location', (6,6,5))

    # The guesser needs an initial condition for its localization
    # estimates.  For most applications, you'll want to use as wide
    # and uniform a distribution as possible.  Here, we've chosen a
    # uniform distribution covering the room we use for most of our
    # experiments, in the coordinate system we've defined for the
    # entire floor of the building.
    guesser.uniform(npoints, [[28.0, 35.0], [35.0, 40.0], [-math.pi, math.pi]])

    # If you're doing simulation experiments, or if there is some
    # other reason you want to give the guesser a head start, you can
    # use a normal distribution instead.
    # guesser.normal([[0.0,0.01], [0.0, 0.01], [0.0, 0.01]])

    server = PerfesserServer(rospy.ServiceProxy, debug=True)

    rospy.Subscriber('guesser/position/guessers', Announce, server.updateStack)

    rospy.sleep(5.0) # wait for the subscriber to be set up

    pts = Belief()
    pts.sender = "perfesser"
    pts.data_type = "location"
    pts.pers = (0.0, 0.0, 2 * math.pi)
    counter = 0

    while True:
        r = server.surveyGuess(guesser)

        if not r.no_data:
            pts.points = guesser.outPoints()
            pts.means = guesser.means()
            pts.stds = guesser.stds()
            pts.source_stamp = guesser.stamp
            pts.source_data = guesser.source_data
            pts.data_type = "location"
            pts.header.stamp = rospy.Time.now()
            counter += 1
            pub.publish(pts)

        rospy.sleep(wait_time)



    rospy.spin()

