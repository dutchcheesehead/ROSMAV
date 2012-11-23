#!/usr/bin/env python
#
# This is the program that runs the coverage controller for a single robot.
import roslib ; roslib.load_manifest('cover')
import rospy
from perfesser.msg import Pt, Belief
from cover.srv import MRFQuery, MRFQueryRequest, MRFQueryResponse
from irobot_create_2_1a.msg import SensorPacket
import coverage
import socket

if __name__ == "__main__":

    away = rospy.get_param('/cover/kAway', 1.3)
    space = rospy.get_param('/cover/kSpace', 1.3)
    avoid = rospy.get_param('/cover/kAvoid', 1.3)
    seek = rospy.get_param('/cover/kSeek', 1.3)
    maxDist = rospy.get_param('/cover/maxDist', 1.3)

    machineString = rospy.get_param('/cover/machines', "georgia,tahoma")
    machines = machineString.split(",")

    print "machines:", machines

    rospy.init_node("cover")

#    velPub = rospy.Publisher("/cmd_vel", Twist)


    goalPub = rospy.Publisher("cover/goal", Pt)

    ccontrol = coverage.CoverageControl(socket.gethostname(), 
                                        npoints=40,
                                        away = away,
                                        space = space,
                                        avoid = avoid, 
                                        seek = seek,
                                        maxDist = maxDist,
                                        goalPub = goalPub)
    rospy.sleep(1.0)

    sensorSub = rospy.Subscriber("sensorPacket", SensorPacket, 
                                          ccontrol.trackSensors)

    posSub = rospy.Subscriber("guesser/position/guess", 
                              Belief,
                              ccontrol.trackPos,
                              queue_size = 1, buff_size = 250)

    advice = rospy.Service("cover/advice", MRFQuery, 
                           ccontrol.takeAdvice)

#   roomLoc = rospy.Subscriber("olsr/neighbors", Room, 
#                              ccontrol.recordNeighbors)
#    roomLoc = rospy.Subscriber("/odom", Odometry,
#                               ccontrol.recordNeighbors)



    # This is a hack until the ROS node exists.
    ccontrol.recordNeighbors(machines)

    print "cover node started, list: <", ccontrol.neighbors[0].name, ">"

    rospy.spin()
