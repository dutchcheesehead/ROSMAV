#!/usr/bin/env python
import roslib; roslib.load_manifest('step_nav')
import rospy
import math
import random
import traceback
from geometry_msgs.msg import Twist, Pose
from position_tracker.msg import Position
from perfesser.msg import Belief
from irobot_create_2_1a.srv import PlaySong
from irobot_create_2_1a.msg import SensorPacket
from goal import *
from force import *
from step_nav.msg import Fudge
from step_to_goal import *

if __name__ == '__main__':

    rospy.init_node('step_nav')
    pub = rospy.Publisher('cmd_vel', Twist)
    playSong = rospy.ServiceProxy('play', PlaySong)

    targetPub = rospy.Publisher('/model_create/placeTarget', Pose)

    def victory(pos):
        print "HAHAHAHA!", str(pos)
        playSong(10) # (int(random.random() * 10))

    def signalFn(n):
        playSong(n)


    stg = StepToGoal((0.0,0.0,2*math.pi), pub, 0.15, 0.3, 0.5, 0.05,
                     stdthres=1.5, debug=True, targetFn=victory,
                     dbgdir="/home/obot/steplog", signalFn=signalFn)
    rospy.Subscriber('guesser/position/guess', Belief, stg.updateLocation)
#    rospy.Subscriber('/position', Position, stg.updateLocation)

    stg.setFudge(Fudge(fudge=0.8))
    rospy.Subscriber('fudge', Fudge, stg.setFudge)

    rospy.Subscriber('sensorPacket', SensorPacket, stg.perturbGoal,
                     queue_size = 1, buff_size = 256)

    import signal
    import sys
    def handler(signum, frame):
        print "....all right already, I'm stopping.  Jeez what a nag."
        raise

    signal.signal(signal.SIGINT, handler)

    rospy.sleep(1.0)

    try:

#        stg.pushGoal(goal.Goal((26.95, 35.60, 2.106)))
#        stg.pushGoal(goal.Goal((31.97, 35.73, -1.863)))
#        stg.pushGoal(goal.Goal((1.5,1.75,0.0)))
#        stg.pushGoal(goal.Goal((0.0,-0.75,0.0)))
#        stg.addForce("F",(32.0, 37.0, 0.0))
#        p = Pose()
#        p.position.x =  33.0
#        p.position.y =  38.0
#        targetPub.publish(p)
        stg.pushGoal(goal.Goal((33.0, 38.0, 0.0)))
        stg.pushGoal(goal.Goal((31.0, 36.0, 0.0)))
        stg.pushGoal(goal.Goal((30.5, 39.0, 0.0)))


# Down the hall on first floor
        # stg.pushGoal(goal.Goal([12.93, 17.71, 0.0]))
        # stg.pushGoal(goal.Goal([16.137, 20.57, -2.35619]))
        # stg.pushGoal(goal.Goal([20.58, 20.81, -3.14159]))
        # stg.pushGoal(goal.Goal([27.73, 19.71, -4.71239]))
        # stg.pushGoal(goal.Goal([30.73, 19.91, -1.5707926]))
        # stg.pushGoal(goal.Goal([30.58, 20.61, 0.0]))
        # stg.pushGoal(goal.Goal([30.73, 26.31, -3.1415926]))
        # stg.pushGoal(goal.Goal([30.58, 30.51, 0.0]))
        # stg.pushGoal(goal.Goal([31.0, 35.0, -1.5]))
        # stg.pushGoal(goal.Goal([32.0, 35.85, -1.570792]))

        # stg.addForce("F", (32.0,  34.85, 1.5708))
        # stg.addForce("F", (31.58, 30.51, 3.1416))
        # stg.addForce("F", (29.73, 26.31, 0.0))
        # stg.addForce("F", (31.58, 20.61, 3.1416))
        # stg.addForce("F", (30.73, 18.91, 1.5708))
        # stg.addForce("F", (25.73, 20.71,-1.5708))
        # stg.addForce("F", (19.58, 20.81, 0.0))
        # stg.addForce("F", (15.43, 19.86, 0.7854))
        # stg.addForce("F", (13.93, 17.71, 3.1416))

# A shorter course
#        stg.pushGoal(goal.Goal([26.83, 34.76, -3.14159]))
#        stg.pushGoal(goal.Goal([27.68, 33.26, 1.5708]))
#        stg.pushGoal(goal.Goal([29.58, 31.71, -1.5708]))
#        stg.pushGoal(goal.Goal([30.58, 30.51, 0.0]))
#        stg.pushGoal(goal.Goal([30.8, 33.3, 0.0]))
#        stg.pushGoal(goal.Goal([31.0, 35.0, -1.5]))
#        stg.pushGoal(goal.Goal([32.0, 35.85, -1.5708]))


 #       stg.addForce("F", (32.0, 34.85, 1.5708))
 #       stg.addForce("F", (31.58,30.51,3.1416))
 #       stg.addForce("F", (29.58,30.71,1.5708))
 #       stg.addForce("F", (27.68,34.26,-1.5708))
 #       stg.addForce("F", (25.83,34.76,0.0))

        signalFn(14)


        while not rospy.is_shutdown():
            if stg.achievedGoal() and (not stg.gs.top()):
                raise Exception("Let me out of here!")
            rospy.sleep(0.5)

#        rospy.spin()

    except:
        print "*************EXIT*************"
        print sys.exc_type
        print sys.exc_value
        traceback.print_tb(sys.exc_traceback)

    finally:
        twist = Twist()
        pub.publish(twist)
