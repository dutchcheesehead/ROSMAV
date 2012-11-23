import sys
import roslib; roslib.load_manifest('ik_control')
import rospy
import ros
from arm_movearm import Arm
from ros import rosservice
from geometry_msgs.msg import PoseStamped
from threading import Lock
from time import sleep


def handlePose(msg):
    global lock
    #check to see if we are already moving due to some message
    if(lock.acquire(0)):
        print "trying"
        Larm.goToPoint([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])   
        lock.release()
    return

if __name__ == "__main__":
    global lock
    print "starting"

    lock=Lock()

    
    rospy.init_node('ik_control')
    
    print "here"
    Larm=Arm("move_left_arm", "left_arm", "ompl_planning/plan_kinematic_path")
    
    rospy.Subscriber("/l_arm_ik_req", PoseStamped, handlePose, queue_size=1)
  
    rospy.spin()  
    Larm.goToPoint([.4, .2, .2], [0,0,0,1])
    sleep(5)
    print "1"
    Larm.goToPoint([.2, .2, .2], [0,0,0,1])
    sleep(5)
    print "2"
    Larm.goToPoint([.5, .4, .2], [0,0,0,1])
    sleep(5)
    print "3"
    Larm.goToPoint([.6, .6, .2], [0,0,0,1])

    
