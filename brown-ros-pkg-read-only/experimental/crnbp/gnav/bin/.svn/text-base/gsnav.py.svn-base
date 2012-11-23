#!/usr/bin/env python
import roslib; roslib.load_manifest('gsnav')
import rospy
import math
import random
import traceback
from geometry_msgs.msg import Twist, Pose
from ar_filter.msg import Lmark, Lmarks
from perfesser.msg import Belief
from irobot_create_2_1a.srv import PlaySong
from irobot_create_2_1a.msg import SensorPacket
from planar_tracker.msg import Tag_Position, Tag_Positions
from goal import *
from force import *
from gs_nav import *

def purgeTags(lpub):
    lpub.publish(Lmarks(operation="purge",sender="gsnav"))

def addTag(lpub, idx, loc):
    out = Lmarks(operation="add",sender="gsnav")
    out.lm.append(Lmark(id=idx,loc=loc))
    gn.addForce("R",loc)
    lpub.publish(out)

def inroom(lpub):
    # alpha - chair next to desk
    addTag(lpub, 0, (32.0, 34.85, 1.5708))
    # delta - over to right when standing with back to door.
    addTag(lpub, 3, (35.0, 39.0,  3.1415))
    # Y - over to left
    addTag(lpub, 9, (29.5, 39.0,  0.0))

    print ">>>>>>>> IN ROOM <<<<<<<<<<"
    gn.pushGoal(goal.Goal((33.0, 38.0, 0.0)))
    gn.pushGoal(goal.Goal((31.0, 36.0, 0.0)))
    gn.pushGoal(goal.Goal((30.5, 39.0, 0.0)))



# Down the hall on first floor
def downhallfirst(lpub):
    print ">>>>>>>> DOWN HALL <<<<<<<<<<"

    purgeTags(lpub)
    # alpha - Chair next to desk
    addTag(lpub,0, (32.0, 34.85, 1.5708))
    # beta - next to other lab door
    addTag(lpub,1, (31.58,30.51, 3.1416))
    # cents, corner opposite door, by stairwell
    addTag(lpub,2, (29.58,30.71, 1.5708))
    # delta = near corner of loading dock door
    addTag(lpub,3, (31.58,20.61, 3.1416))
    # epsilon = end of hall, corner facing lab
    addTag(lpub,4, (30.73,18.91, 1.5708))
    # eta = 4 m down hall from corner
    addTag(lpub,5, (25.73,20.71, -1.5708))
    # gamma = coke machine, outside corner, facing loading dock
    addTag(lpub,6, (19.58,20.81, 0.0))
    # iota = open door across from vending machine, near hinge
    addTag(lpub,8, (15.43,19.86, .7854))
    # kappa = outside wall, 3m back toward lab from delta
    addTag(lpub,10, (31.58,23.61, 3.1416))
    # zeta - opposite second door, farther corner from lab
    addTag(lpub,12, (29.73,26.31, 0.0))

    gn.pushGoal(goal.Goal([12.93, 17.71, 0.0]))
    gn.pushGoal(goal.Goal([16.137, 20.57, -2.35619]))
    gn.pushGoal(goal.Goal([20.58, 19.81, -3.14159]))
    gn.pushGoal(goal.Goal([22.58, 19.81, -3.14159]))
    gn.pushGoal(goal.Goal([25.58, 19.81, -3.14159]))
    gn.pushGoal(goal.Goal([27.73, 19.71, -4.71239]))
    gn.pushGoal(goal.Goal([28.73, 19.71, -4.71239]))
    gn.pushGoal(goal.Goal([29.73, 19.71, -4.71239]))
    gn.pushGoal(goal.Goal([30.73, 19.91, -1.5707926]))
    gn.pushGoal(goal.Goal([30.58, 20.61, 0.0]))
    gn.pushGoal(goal.Goal([30.58, 23.61, 0.0]))
    gn.pushGoal(goal.Goal([30.73, 26.31, -3.1415926]))
    gn.pushGoal(goal.Goal([30.58, 28.51, 0.0]))
    gn.pushGoal(goal.Goal([30.58, 30.51, 0.0]))
    gn.pushGoal(goal.Goal([31.0, 35.0, -1.5]))
    gn.pushGoal(goal.Goal([32.0, 35.85, -1.570792]))

def downhallfirstshort(lpub):
# A shorter course
    print ">>>>>>>> DOWN HALL (SHORT COURSE) <<<<<<<<<<"

    purgeTags(lpub)
    # alpha - Chair next to desk
    addTag(lpub, 0, (32.0, 34.85, 1.5708))
    # beta - next to other lab door
    addTag(lpub, 1, (31.58,30.51, 3.1416))
    # cents, corner opposite door, by stairwell
    addTag(lpub, 2, (9.58,30.71,1.5708))
    # delta -- corner to right of lab door
    addTag(lpub, 3, (7.68,34.26, -1.5708))
    # epsilon -- deep corner to right
    addTag(lpub, 4, (5.83,34.76, 0.0))

    gn.pushGoal(goal.Goal([26.83, 34.76, -3.14159]))
    gn.pushGoal(goal.Goal([27.68, 33.26, 1.5708]))
    gn.pushGoal(goal.Goal([29.58, 31.71, -1.5708]))
    gn.pushGoal(goal.Goal([30.58, 30.51, 0.0]))
    gn.pushGoal(goal.Goal([30.8, 33.3, 0.0]))
    gn.pushGoal(goal.Goal([31.0, 35.0, -1.5]))
    gn.pushGoal(goal.Goal([32.0, 35.85, -1.5708]))


rospy.init_node('gsnav')

pub = rospy.Publisher('cmd_vel', Twist)
lpub = rospy.Publisher("/guesser/position/landmarks", Lmarks)
playSong = rospy.ServiceProxy('play', PlaySong)

def victory(pos):
    print "HAHAHAHA!", str(pos)
    playSong(10) # (int(random.random() * 10))

def signalFn(n):
    playSong(n)

gn = GuessNav(dims=(0.0,0.0,2*math.pi),
              publisher=pub,
              maxSpeed=0.2,
              maxTurn=0.5,
              dampSpeed=0.75,
              dampTurn=0.75,
              kSpeed = 1.0,
              kTurn = 1.0,
              stdThres=2.75,
              zeroDistance=0.4,
              targetFn=victory,
              signalFn=signalFn,
              debug=True,
              dbgdir="/home/obot/steplog")

rospy.Subscriber('/guesser/position/guess', Belief, gn.doMove)
rospy.Subscriber('sensorPacket', SensorPacket, gn.perturbGoal,
                 queue_size = 1, buff_size = 256)
rospy.Subscriber('/verdana/tag_positions', Tag_Positions, gn.getRealRobotPos)

import signal
import sys
def handler(signum, frame):
    print "....all right already, I'm stopping.  Jeez what a nag."
    # signalFn(6)
    raise

signal.signal(signal.SIGINT, handler)

rospy.sleep(1.0)

try:

    inroom(lpub)
#    downhallfirstshort(lpub)
#    downhallfirst(lpub)

#    signalFn(2)

    while not rospy.is_shutdown():
        if gn.achievedGoal() and (not gn.gs.top()):
            raise Exception("Let me out of here!")
        rospy.sleep(0.5)


except:
    print "*************EXIT*************"
    print sys.exc_type
    print sys.exc_value
    traceback.print_tb(sys.exc_traceback)

finally:
    twist = Twist()
    pub.publish(twist)

