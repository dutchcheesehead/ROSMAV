#!/usr/bin/env python
#
import roslib ; roslib.load_manifest('cover')
import rospy
import numpy as np
from model_room import TwoDRobotModel, TwoDRoomModel
from cover.msg import Room, Room2D


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

    room = TwoDRoomModel(0.1, 650, neighborDistance=3.2,
                         perimeter = [[0.0132450331125828, 7.37748344370861],
                                      [0.582781456953642, 7.94701986754967],
                                      [0.90066225165563, 7.82781456953642],
                                      [1.28476821192053, 7.86754966887417],
                                      [1.28476821192053, 7.37748344370861],
                                      [3.44370860927152, 7.37748344370861],
                                      [3.44370860927152, 7.17880794701987],
                                      [3.82781456953642, 7.17880794701987],
                                      [3.82781456953642, 7.37748344370861],
                                      [4.17218543046358, 7.37748344370861],
                                      [4.17218543046358, 7.11258278145695],
                                      [3.96026490066225, 7.11258278145695],
                                      [3.96026490066225, 6.59602649006622],
                                      [4.17218543046358, 6.59602649006622],
                                      [4.17218543046358, 5.52317880794702],
                                      [4.7682119205298, 5.52317880794702],
                                      [4.7682119205298, 4.99337748344371],
                                      [5.54966887417219, 4.99337748344371],
                                      [5.54966887417219, 4.56953642384106],
                                      [4.19867549668874, 4.56953642384106],
                                      [4.18543046357616, 4.35761589403973],
                                      [4.18543046357616, 4.31788079470199],
                                      [4.75496688741722, 4.31788079470199],
                                      [4.75496688741722, 4.13245033112583],
                                      [5.58940397350993, 4.13245033112583],
                                      [5.58940397350993, 4.34437086092715],
                                      [5.74834437086093, 4.34437086092715],
                                      [5.74834437086093, 4.99337748344371],
                                      [6.34437086092715, 4.96688741721854],
                                      [6.33112582781457, 4.13245033112583],
                                      [8.23841059602649, 4.13245033112583],
                                      [8.23841059602649, 6.75496688741722],
                                      [7.88079470198676, 6.75496688741722],
                                      [7.88079470198676, 7.09933774834437],
                                      [7.24503311258278, 7.09933774834437],
                                      [7.24503311258278, 7.90728476821192],
                                      [7.6158940397351, 7.90728476821192],
                                      [7.6158940397351, 7.54966887417219],
                                      [8.23841059602649, 7.54966887417219],
                                      [8.23841059602649, 7.68211920529801],
                                      [7.73509933774834, 7.68211920529801],
                                      [7.73509933774834, 10.00000],
                                      [9.78807947019868, 10.00000],
                                      [9.74834437086093, 9.72185430463576],
                                      [9.90728476821192, 9.72185430463576],
                                      [9.90728476821192, 8.13245033112583],
                                      [9.68211920529801, 8.13245033112583],
                                      [9.68211920529801, 7.54966887417219],
                                      [8.7682119205298, 7.54966887417219],
                                      [8.7682119205298, 7.68211920529801],
                                      [8.63576158940397, 7.68211920529801],
                                      [8.63576158940397, 7.4569536423841],
                                      [9.96026490066225, 7.4569536423841],
                                      [9.96026490066225, 7.13907284768212],
                                      [8.72847682119205, 7.13907284768212],
                                      [8.72847682119205, 3.28476821192053],
                                      [8.52980132450331, 3.28476821192053],
                                      [8.52980132450331, 3.65562913907285],
                                      [4.59602649006623, 3.65562913907285],
                                      [4.4635761589404, 3.85430463576159],
                                      [4.19867549668874, 3.85430463576159],
                                      [4.19867549668874, 3.57615894039735],
                                      [3.50993377483444, 3.57615894039735],
                                      [3.50993377483444, 3.20529801324503],
                                      [2.92715231788079, 3.20529801324503],
                                      [2.92715231788079, 1.86754966887417],
                                      [2.47682119205298, 1.86754966887417],
                                      [2.43708609271523, 3.84105960264901],
                                      [0.370860927152318, 3.81456953642384],
                                      [0.370860927152318, 4.31788079470199],
                                      [0.649006622516556, 4.35761589403973],
                                      [0.649006622516556, 6.71523178807947],
                                      [0.132450331125828, 6.87417218543046],
                                      [0.0132450331125828, 7.37748344370861]])

#    point: [8.7360506057739258, 7.1292200088500977]

    room.pause("stop")

    robots = []
    def addRobot(name, xloc, yloc, color, p):
        robots.append(TwoDRobotModel(name, xloc, yloc, color,
                                     perimeter = p, radius = 0.06))
        room.addRobot(robots[-1])
        robots[-1].busy=True

    sp = lambda(x): x + (np.random.random()-0.5)/8.0

    addRobot("umberto", sp(8.5), sp(4.8), 100.0, room.perimeter)
    addRobot("tymon", sp(8.5), sp(4.9), 95.0, room.perimeter)
    addRobot("sam", sp(8.5), sp(5.0), 90.0, room.perimeter)
    addRobot("roger", sp(8.5), sp(5.1), 85.0, room.perimeter)
    addRobot("quincy", sp(8.5), sp(5.2), 80.0, room.perimeter)
    addRobot("peter", sp(8.5), sp(5.3), 75.0, room.perimeter)
    addRobot("oswald", sp(8.5), sp(5.4), 70.0, room.perimeter)
    addRobot("neville", sp(8.5), sp(5.5), 65.0, room.perimeter)
    addRobot("mark", sp(8.5), sp(5.6), 60.0, room.perimeter)
    addRobot("louis", sp(8.5), sp(5.7), 55.0, room.perimeter)
    addRobot("kenny", sp(8.5), sp(5.8), 50.0, room.perimeter)
    addRobot("joseph", sp(8.5), sp(5.9), 45.0, room.perimeter)
    addRobot("ivan", sp(8.5), sp(6.0), 40.0, room.perimeter)
    addRobot("harold", sp(8.5), sp(6.1), 35.0, room.perimeter)
    addRobot("george", sp(8.5), sp(6.2), 30.0, room.perimeter)
    addRobot("fox", sp(8.5), sp(6.3), 25.0, room.perimeter)
    addRobot("easy", sp(8.6), sp(6.4), 20.0, room.perimeter)
    addRobot("dog", sp(8.5), sp(6.5), 15.0, room.perimeter)
    addRobot("charlie", sp(8.5), sp(6.7), 10.0, room.perimeter)
    addRobot("baker", sp(8.5), sp(6.7), 5.0, room.perimeter)
    addRobot("abel", sp(8.5), sp(6.8), 0.0, room.perimeter)

    locpub = rospy.Publisher("/room/locations", Room2D)
    dbgpub = rospy.Publisher("/room/locations_debug", Room)

    for rb in robots:
        rb.busy=False

    try:

        start = rospy.Time.now().to_sec()
        startd = 99915.0
        startg = 99916.0
        starth = 99917.0
        starti = 99918.0
        startj = 99925.0

#        rospy.sleep(20.0)
        room.pause("start")
        r = rospy.Rate(2)
        while True:
            room.updateRoom()
            room.drawConnections()
            if start + startd < rospy.Time.now().to_sec():
                room.delRobot("dog")

            if start + startg < rospy.Time.now().to_sec():
                addRobot("george", 0.5, 0.5, 95.0, room.perimeter)
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
            dbgpub.publish(room.list())
            r.sleep()

    except:
        print "*************EXIT*************"
        print sys.exc_type
        print sys.exc_value
        traceback.print_tb(sys.exc_traceback)
