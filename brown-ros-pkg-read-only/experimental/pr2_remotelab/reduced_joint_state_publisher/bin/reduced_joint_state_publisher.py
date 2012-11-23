#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service

import roslib
roslib.load_manifest('reduced_joint_state_publisher')
import rospy
from reduced_joint_state_publisher.srv import *
from sensor_msgs.msg import JointState

class LatestJointStates:
    def __init__(self, pub, joints):
        self.pub=pub
        self.joints=joints
        self.last_message=[]
        
        rospy.Subscriber('joint_states', JointState, self.joint_states_listener, queue_size=1)
        s=rospy.Service('set_requested_joints', RequestedJointStates, self.set_joints)
        
        
    def set_joints(self, req):
       temp_names=[] 
       item_indicator=[]
       for name in req.names:
           if name in self.last_message.name:
               temp_names.append(name)
               item_indicator.append(True)
           else:
               item_indicator.append(False)
           
           self.joints=temp_names
       return RequestedJointStatesResponse(item_indicator)
 
    def joint_states_listener(self, msg):
        #print msg
        self.last_message=msg
        reduced_msg=JointState()
        reduced_msg.name=self.joints

        for joint_name in self.joints:
            reduced_index=self.joints.index(joint_name)
            global_index=msg.name.index(joint_name)
            reduced_msg.position.insert(reduced_index, msg.position[global_index])
            reduced_msg.velocity.insert(reduced_index, msg.velocity[global_index])
            reduced_msg.effort.insert(reduced_index, msg.effort[global_index])
        self.pub.publish(reduced_msg)

if __name__ == "__main__":
    rospy.init_node('reduced_joint_states_publisher')
    pub=rospy.Publisher('requested_joint_states', JointState)
    
    joints=rospy.get_param('requested_joint_names', ["l_shoulder_pan_joint",
                         "l_shoulder_lift_joint",
                         "l_upper_arm_roll_joint",
                         "l_elbow_flex_joint",
                         "l_forearm_roll_joint",
                         "l_wrist_flex_joint",
                        "l_wrist_roll_joint"])


    latestjointstates = LatestJointStates(pub, joints)

    print "joints_states_publisher server started, waiting for queries"
    rospy.spin()
