#!/usr/bin/env python
import roslib; roslib.load_manifest('move')
import rospy
from geometry_msgs.msg import Twist



def go():
    pub = rospy.Publisher('cmd_vel',Twist)
    rospy.init_node('move')

    s = rospy.myargv()[1:]

    #publish and quit
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    rospy.logerr("test")
    if len(s) == 1:
        s = s[0]
        if s == "forward":
            rospy.logerr("test")
            twist.linear.x = 1
            pub.publish(twist)
        elif s == "backward":
            twist.linear.x = -1
            pub.publish(twist)
        elif s == "left":
            twist.angular.z = 1
            pub.publish(twist)
        elif s == "right":
            twist.angular.z = -1
            pub.publish(twist)
        elif s == "stop":
            pub.publish(twist)
    return



def dtrace(s,x):
    rospy.loginfo(s+" == %s"%x)
    return x

if __name__ == '__main__':
    go()



