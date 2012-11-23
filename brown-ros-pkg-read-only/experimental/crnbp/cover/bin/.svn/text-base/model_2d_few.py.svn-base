#!/usr/bin/env python
#
import roslib ; roslib.load_manifest('cover')
import rospy
from model_room import TwoDRoomModel, TwoDRobotModel
from cover.msg import Room, Room2D
import numpy as np


if __name__ == '__main__':
    rospy.init_node('cover')
    import time
    import signal
    import sys
    import traceback

    def handler(signum, frame):
        print "....all right already, I'm stopping.  Jeez what a nag."
        raise

    signal.signal(signal.SIGINT, handler)

    room = TwoDRoomModel(0.1, 500, neighborDistance=2.0,
                         perimeter = [[0.0,0.0],[0.0,10.0],[7.,10.0],
                                      [7.,5.],[10.0,5.],[10.0,2.5],
                                      [7.,2.5],[7.,0.0],
                                      [0.0,0.0]])

    robots = []
    def addRobot(name, xloc, yloc, color, p):
        robots.append(TwoDRobotModel(name, xloc, yloc, color, perimeter = p))
        room.addRobot(robots[-1])

    sp = lambda(x): x + (np.random.random()-0.5)/1.5

    addRobot("abel", sp(5.), sp(5.), 0.0, room.perimeter)
    addRobot("baker", sp(5.), sp(5.), 20.0, room.perimeter)
    addRobot("charlie", sp(5.), sp(5.), 40.0, room.perimeter)
    addRobot("dog", sp(5.), sp(5.), 60.0, room.perimeter)
    addRobot("easy", sp(5.), sp(5.), 80.0, room.perimeter)
    addRobot("fox", sp(5.), sp(5.), 70.0, room.perimeter)
    addRobot("george", sp(5.), sp(5.), 50.0, room.perimeter)
    addRobot("harold", sp(5.), sp(5.), 100.0, room.perimeter)
    # addRobot("ivan", sp(5.), sp(5.), 30.0, room.perimeter)
    # addRobot("joseph", sp(5.), sp(5.), 100.0, room.perimeter)
    # addRobot("kenny", sp(5.), sp(5.), 100.0, room.perimeter)
    # addRobot("louis", sp(5.), sp(5.), 100.0, room.perimeter)
    # addRobot("mark", sp(5.), sp(5.), 100.0, room.perimeter)
    # addRobot("neville", sp(5.), sp(5.), 100.0, room.perimeter)
    # addRobot("otto", sp(5.), sp(5.), 100.0, room.perimeter)
    # addRobot("peter", sp(5.), sp(5.), 100.0, room.perimeter)

    locpub = rospy.Publisher("/room/locations", Room2D)

    try:

        start = rospy.Time.now().to_sec()
        startd = 9960.0
        startg = 9120.0
        starth = 9917.0
        starti = 9918.0
        startj = 9985.0


        r = rospy.Rate(4)
        while True:
            room.updateRoom()
            room.drawConnections()
            if start + startd < rospy.Time.now().to_sec():
                room.delRobot("dog")

            if start + startg < rospy.Time.now().to_sec():
                addRobot("doggy", 0.5, 0.5, 95.0, room.perimeter)
                startg = 10000000

            if start + starth < rospy.Time.now().to_sec():
                addRobot("how", 0.2, 0.5, 100.0, room.perimeter)
                starth = 10000000

            if start + starti < rospy.Time.now().to_sec():
                addRobot("ivy", 0.8, 0.5, 100.0, room.perimeter)
                addRobot("joseph", 0.81, 0.4, 100.0, room.perimeter)
                addRobot("kappa", 0.82, 0.3, 100.0, room.perimeter)
                starti = 10000000

            if start + startj < rospy.Time.now().to_sec():
                room.delRobot("george")
                room.delRobot("how")
                room.delRobot("ivy")
                room.delRobot("joseph")
                startj = 10000000

            locpub.publish(room.nearest())
            r.sleep()

    except:
        print "*************EXIT*************"
        print sys.exc_type
        print sys.exc_value
        traceback.print_tb(sys.exc_traceback)
