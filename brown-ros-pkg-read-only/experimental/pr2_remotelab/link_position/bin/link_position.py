#!/usr/bin/env python

import roslib
roslib.load_manifest('link_position')
import rospy
from geometry_msgs.msg import PoseStamped
import tf

class LinkPositionPublisher:
    def __init__(self, pub):
        self.pub=pub
        self.link_names=[]
        
        s=rospy.Service('set_requested_links', RequestedLinks, self.set_links)

    def set_links(self, req):
        self.frame_id=req.frame_id
        self.link_names=req.child_frames
        
        return RequestedLinksResponse(True)
    
    
    

if __name__=="__main__":
    rospy.init_node('link_position')
    tf_listener = tf.TransformListener()
    pub=rospy.Publisher('l_wrist_link_position', PoseStamped)
    pose=PoseStamped()
    
    while not rospy.is_shutdown():
        
        try:
            currtime=rospy.Time.now()
            
            # tf_listener.waitForTransform('base_link', 'r_wrist_roll_link', currtime, rospy.Duration(0))
            
            tf_frame= tf_listener.lookupTransform('base_link', 'l_wrist_roll_link', rospy.Time(0))
            print tf_frame
            
            pose.pose.position.x=tf_frame[0][0]
            pose.pose.position.y=tf_frame[0][1]
            pose.pose.position.z=tf_frame[0][2]

            pose.pose.orientation.x=tf_frame[1][0]
            pose.pose.orientation.x=tf_frame[1][1]
            pose.pose.orientation.x=tf_frame[1][2]
            pose.pose.orientation.x=tf_frame[1][3]
            
            pub.publish(pose)

        except(tf.LookupException, tf.ConnectivityException, tf.Exception):
            
           
            continue
        
        rospy.sleep(.06)
        
    
