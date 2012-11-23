#this function simply tests the ik by publishing a bunch of messages


import sys
import roslib; roslib.load_manifest('ik_control')
import rospy
import ros
from time import sleep

from geometry_msgs.msg import PoseStamped

if __name__=="__main__":
    
    rospy.init_node('test_ik', anonymous=True)
    pub=rospy.Publisher('l_arm_ik_req', PoseStamped)
    

    #5 messages that should work

    pose= PoseStamped()
    #print dir(pose)
    #print pose.header

    print "Going to position 1"
    pose.pose.position.x =.4
    pose.pose.position.y = .2
    pose.pose.position.z =.2

    pose.pose.orientation.x =0
    pose.pose.orientation.y =0
    pose.pose.orientation.z =0
    pose.pose.orientation.w=1

    pub.publish(pose)

    sleep(2)

    print "Going to position 2"
    pose.pose.position.x =.4
    pose.pose.position.y = .2
    pose.pose.position.z =.2

    pose.pose.orientation.x =0
    pose.pose.orientation.y =0
    pose.pose.orientation.z =0
    pose.pose.orientation.w=1

    pub.publish(pose)

    sleep(2)


    print "Going to position 3"
    pose.pose.position.x =.5
    pose.pose.position.y = .4
    pose.pose.position.z =.2

    pose.pose.orientation.x =0
    pose.pose.orientation.y =0
    pose.pose.orientation.z =0
    pose.pose.orientation.w=1

    pub.publish(pose)

    sleep(2)

    print "Going to position 4"
    pose.pose.position.x =.6
    pose.pose.position.y = .6
    pose.pose.position.z =.2
    pose.pose.orientation.x =0
    pose.pose.orientation.y =0
    pose.pose.orientation.z =0
    pose.pose.orientation.w=1

    pub.publish(pose)

    sleep(2)

    print "Going to position 5"
    pose.pose.position.x =.6
    pose.pose.position.y = .6
    pose.pose.position.z =.2

    pose.pose.orientation.x =0
    pose.pose.orientation.y =0
    pose.pose.orientation.z =1
    pose.pose.orientation.w=1

    pub.publish(pose)

    sleep(2)

    
