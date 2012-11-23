#!/usr/bin/env python

# Receives velocity commands and relays that information along, so long as
#   the robot is not touching a (high light intensity) border.  When a border
#   is reached, this controller reverses the last issued command until the
#   robot is off of the high-intensity line, thereby preventing escape.

import roslib; roslib.load_manifest('virtual_border')
import rospy, sys
from geometry_msgs.msg import Twist
#from nxt_msgs.msg import Color
from irobot_create_2_1.msg import SensorPacket


intensity = False
twist = False
lastMovingTwist = False
update = False
onBorder = False

def receiveSensorPacket(sp):
    global intensity, update
    intensity = sp.cliffFrontLeftSignal
    update = True

def receiveTwist(newTwist):
    global twist, lastMovingTwist, onBorder, update
    update = True
    # can only control if not on line
    if not onBorder:
        twist = newTwist
        if newTwist.linear.x != 0 or newTwist.angular.z != 0:
            lastMovingTwist = newTwist

def go():
    global intensity, twist, lastMovingTwist, update, onBorder
    # init
    args = rospy.myargv(sys.argv)
    if (len(args) != 3 or
        (args[1] != "--border-is-brighter" and args[1] != "--border-is-darker")):
        rospy.logerr("Usage: " + args[0] + "{--border-is-brighter,--border-is-darker} [threshold]")
        sys.exit(2)

    borderIsDarker = (args[1] == "--border-is-darker")
    THRESHOLD = float(args[2])
    pub = rospy.Publisher('cmd_vel',Twist)
    rospy.init_node('virtual_border')
    rospy.Subscriber('cmd_vel_unregulated', Twist, receiveTwist)
    rospy.Subscriber('sensorPacket', SensorPacket, receiveSensorPacket)
    # constants
    reversing = False
    # main loop
    while not rospy.is_shutdown():
        while not update: pass
        update = False;
        onBorder = (intensity and
                    ((intensity < THRESHOLD) if borderIsDarker else (intensity > THRESHOLD)))
        print str(THRESHOLD) + " " + str(onBorder) + " " + str(intensity)
        if not onBorder:
            # let things be
            reversing = False
            if twist:
                pub.publish(twist)
        elif reversing == True:
            pass
        else:
            # Just landed on a border... runaway robot!
            reversing = True
            reverseMotion = False
            if lastMovingTwist:
                reverseMotion = lastMovingTwist
                reverseMotion.linear.x = -reverseMotion.linear.x
                reverseMotion.angular.z = -reverseMotion.angular.z
                # rospy.loginfo("lastMovingTwist: " + lastMovingTwist)
                # rospy.loginfo("reverseMotion: " + reverseMotion)
            else:
                reverseMotion = Twist()
                reverseMotion.linear.x = 0
                reverseMotion.linear.y = 0
                reverseMotion.linear.z = 0
                reverseMotion.angular.x = 0
                reverseMotion.angular.y = 0
                reverseMotion.angular.z = 0
            pub.publish(reverseMotion)

if __name__ == '__main__':
    go()


