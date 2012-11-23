#!/usr/bin/env python
import roslib; roslib.load_manifest('position_tracker')
import rospy
import tf
from position_tracker.srv import *
from position_tracker.msg import Position
from math import sin, cos

class Tracker:
    def __init__(self):
        self.offset_x = 0
        self.offset_y = 0
        self.offset_theta = 0
        self.delay = rospy.Duration(0)
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('position', Position)
        self.odom_frame = rospy.get_param('odom_frame', 'odom')
        self.robot_frame = rospy.get_param('robot_frame', 'base_link')

    def translate(self):

        self.broadcaster.sendTransform((self.offset_x, self.offset_y, 0),
                                       tf.transformations.quaternion_from_euler(0, 0, self.offset_theta),
                                       rospy.Time.now() - self.delay,
                                       self.odom_frame,
                                       'world')
        #Yes, this is completely redundant to the transform publisher.
        #But sometimes you don't want to have to learn everything
        #about how tf frames work, you just want to know where your
        #robot is at.

        try:
            now = rospy.Time.now()
#            self.listener.waitForTransform('world', self.robot_frame, now, rospy.Duration(1.0))
            ((x,y,z), rot) = self.listener.lookupTransform('world', self.robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            return
        position = Position()
        position.x = x
        position.y = y
        (phi, psi, position.theta) = tf.transformations.euler_from_quaternion(rot)
        self.pub.publish(position)

    def set_position(self, req):
        try:
            self.delay = rospy.Duration(req.delay)
            now = rospy.Time.now() - self.delay
            self.listener.waitForTransform(self.odom_frame, self.robot_frame, now, rospy.Duration(1.0))
            ((x,y,z), rot) = self.listener.lookupTransform(self.odom_frame, self.robot_frame, now)
        except (tf.LookupException, tf.ConnectivityException):
            return SetPositionResponse(False)

        (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
        self.offset_theta = req.theta - theta
        self.offset_x = (x*cos(self.offset_theta)) - (y*sin(self.offset_theta))
        self.offset_y = (x*sin(self.offset_theta)) + (y*cos(self.offset_theta))

        self.offset_x = req.x - self.offset_x
        self.offset_y = req.y - self.offset_y
        return SetPositionResponse(True)

if __name__ == '__main__':
    rospy.init_node('position_tracker')

    tracker = Tracker()
    
    rospy.Service('set_position', SetPosition, tracker.set_position)

    rate = rospy.Rate(15.0)

    while not rospy.is_shutdown():
        tracker.translate()
        rate.sleep()
