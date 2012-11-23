#!/usr/bin/env python
import roslib; roslib.load_manifest('bag_and_tag')
import rospy
import orange
import orngTree
import time

from geometry_msgs.msg import Twist
from irobot_create_2_1.msg import SensorPacket
from ar_recog.msg import Tags, Tag

class Driver:
    def __init__(self):
        self.last_tag_seen = '0'
        self.tag_visible = 'f'
        self.tag_x_coord = 180.0
        self.tag_distance = 2000.0
        self.bumping = 'f'
        self.data = orange.ExampleTable("analyzed_data/tag_data")
        self.tree = orngTree.TreeLearner(self.data, maxMajority = 0.7)
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self.picked_up = True
        self.generating_tree = False

    def handle_tags(self, msg):
        if len(msg.tags) == 0:
            self.tag_visible='f'
        else:
            self.last_tag_seen = str(msg.tags[0].id)
            self.tag_x_coord = msg.tags[0].x
            self.tag_distance = msg.tags[0].distance
            self.tag_visible='t'

    def handle_sensor_packet(self, msg):
        if (msg.bumpLeft or msg.bumpRight):
            self.bumping = 't'
        else:
            self.bumping = 'f'
        if msg.wheeldropLeft and msg.wheeldropRight:
            self.picked_up = True
        else:
            self.picked_up = False

    def drive(self):
#        if self.generating_tree:
#            return
#        if self.picked_up:
#            self.generating_tree = True
#            data_subset = self.data.select(orange.MakeRandomIndices2(self.data, 0.005), 0)
#            self.tree = orngTree.TreeLearner(data_subset)
#            self.generating_tree = False

        current_observation = \
            orange.Example(self.data.domain, \
                               [self.last_tag_seen, self.tag_visible, \
                                    self.tag_x_coord, self.tag_distance, \
                                    self.bumping, '?'])

        command = self.tree(current_observation).value

        twist = Twist()
        if self.picked_up:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif command == 'f':
            twist.linear.x = 0.25
            twist.angular.z = 0.0
            self.stopped = False
        elif command == 'fr':
            twist.linear.x = 0.25
            twist.angular.z = -1.0
            self.stopped = False
        elif command == 'fl':
            twist.linear.x = 0.25
            twist.angular.z = 1.0
            self.stopped = False
        elif command == 'r':
            twist.linear.x = 0.0
            twist.angular.z = -1.0
            self.stopped = False
        elif command == 'l':
            twist.linear.x = 0.0
            twist.angular.z = 1.0
            self.stopped = False
        elif command == 'b':
            twist.linear.x = -0.25
            twist.angular.z = 0.0
            self.stopped = False
        elif command == 's':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.stopped = True

        self.pub.publish(twist)

if __name__ == '__main__':

    rospy.init_node('learned_driver')

#    rospy.sleep(10)
    driver = Driver()

    rospy.Subscriber('tags', Tags, driver.handle_tags)
    rospy.Subscriber('sensorPacket', SensorPacket, driver.handle_sensor_packet)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        driver.drive()
        rate.sleep()
