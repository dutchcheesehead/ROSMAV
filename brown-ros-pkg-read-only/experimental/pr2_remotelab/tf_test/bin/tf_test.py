#!/usr/bin/env python  
import roslib
roslib.load_manifest('tf_test')
import rospy
import tf
from time import sleep

if __name__ == '__main__':
    rospy.init_node('tf_test_sigh')
    sum=0;
    tf_listener = tf.TransformListener()
   
    #rospy.sleep(1)
    while not rospy.is_shutdown():
        
        try:
            currtime=rospy.Time.now()
                         
           # tf_listener.waitForTransform('base_link', 'r_wrist_roll_link', currtime, rospy.Duration(0))
            
            print tf_listener.lookupTransform('base_link', 'r_wrist_roll_link', rospy.Time(0))
        except(tf.LookupException, tf.ConnectivityException, tf.Exception):
           
            sum=sum+1;
            continue
    
        rospy.sleep(.03)
    print sum
