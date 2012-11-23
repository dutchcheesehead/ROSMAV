#!/usr/bin/env python

import roslib
roslib.load_manifest('twistTranslate')
import rospy
from NAOcontrol.msg import Walk
from geometry_msgs.msg import Twist

global pub

#1 - forward only
#2 - left only
#3 - forward AND left
#4 - right only
#5 - forward AND right

def translate(msg):
    global pub

    motionmsg=Walk()


    if msg.linear.x > 0:
        if msg.angular.z > 0:
            motionmsg.walk = 3
        if msg.angular.z < 0:
            motionmsg.walk = 5
    else:
        if msg.angular.z > 0:
            motionmsg.walk = 2
        if msg.angular.z < 0:
            motionmsg.walk = 4
    
    pub.publish(motionmsg)

if __name__=="__main__":
    global pub

    rospy.init_node("twistTranslator")
    pub=rospy.Publisher("motion", Walk)
    
    rospy.Subscriber("cmd_vel", Twist, translate)

    rospy.spin()
