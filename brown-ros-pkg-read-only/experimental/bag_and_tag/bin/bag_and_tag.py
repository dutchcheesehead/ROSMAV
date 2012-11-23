#!/usr/bin/env python
import roslib; roslib.load_manifest('bag_and_tag')
import rospy
#import subprocess
#import os
#import signal
import time
from bag_and_tag.srv import *
from bag_and_tag.msg import Score
from geometry_msgs.msg import Twist
from irobot_create_2_1.msg import SensorPacket
from ar_recog.msg import Tags, Tag

class Bagger:
    def __init__(self):
        self.in_use = False
        self.ok_to_write = False
        self.start_time = 0.0
        self.bump_count = 0
        self.goal_dist = 1000.0
        self.last_heard_from_operator = 0.0
        self.was_bumping = False
        self.last_tag_id = "-"
        self.last_tag_x = "-"
        self.last_tag_dist = "-" 
        self.last_cmd_vel_x = "-"
        self.last_cmd_vel_z = "-"
        self.last_bump = "-"

    def begin_session(self, req):
        self.start_time = time.time()
        self.was_bumping = False
        self.bump_count = 0
        self.goal_dist = 1000.0
        self.last_heard_from_operator = self.start_time
        datafile_name = "raw_data/" + str(self.start_time)
        if req.mode == 0:
            datafile_name += ".cam"
        else:
            datafile_name += ".tag"
        datafile_name += ".data"
        self.f = open(datafile_name, 'w')
        self.ok_to_write = True
        self.in_use = True
        print("Beginning session...")
        
        return BeginSessionResponse(True)

    def end_session(self, req):
        self.ok_to_write = False
        self.f.close()
        self.in_use = False
        return EndSessionResponse(True)

    def check_session(self, req):
        return CheckSessionResponse(self.in_use)

    def get_score(self, req):
        score = Score()
        score.time = time.time() - self.start_time
        score.distance = self.goal_dist
        score.bumps = self.bump_count
        self.f.close()
        self.ok_to_write = False
        print("Ending session...")
        return GetScoreResponse(score)

    def handle_tags(self, msg):
        if len(msg.tags) == 0:
            self.last_tag_id = "-"
            self.last_tag_x = "-"
            self.last_tag_dist = "-"
        else:
            self.last_tag_id = str(msg.tags[0].id)
            self.last_tag_x = str(msg.tags[0].x)
            self.last_tag_dist = str(msg.tags[0].distance)
            if msg.tags[0].id == 3:
                if msg.tags[0].distance < self.goal_dist:
                    self.goal_dist = msg.tags[0].distance

    def handle_sensor_packet(self, msg):
        if (msg.bumpLeft or msg.bumpRight):
            self.last_bump = "t"
            if not self.was_bumping:
                self.bump_count += 1
                self.was_bumping = True
        else:
            self.was_bumping = False
            self.last_bump = "f"

    def handle_cmd_vel(self, msg):
        self.last_heard_from_operator = time.time()
        self.last_cmd_vel_x = str(msg.linear.x)
        self.last_cmd_vel_z = str(msg.angular.z)

    def print_to_file(self):
        elapsed_time = time.time() - self.last_heard_from_operator
        if self.ok_to_write:
            self.f.write(str(time.time()) + "\t" + self.last_tag_id + "\t" + self.last_tag_x + "\t" + self.last_tag_dist + "\t" + self.last_cmd_vel_x + "\t" + self.last_cmd_vel_z + "\t" + self.last_bump + "\n")
        if self.ok_to_write and time.time() - self.last_heard_from_operator >= 30.0:
            self.ok_to_write = False
            self.f.write("INVALID DATA")
            self.f.close()
            print("Timed out...")
        self.last_tag_id = "-"
        self.last_tag_x = "-"
        self.last_tag_dist = "-"
        self.last_cmd_vel_x = "-"
        self.last_cmd_vel_z = "-"
        self.last_bump = "-"

if __name__ == '__main__':

    rospy.init_node('bag_and_tag')

    bagger = Bagger()

    rospy.Service('begin_session', BeginSession, bagger.begin_session)
    rospy.Service('end_session', EndSession, bagger.end_session)
    rospy.Service('check_session', CheckSession, bagger.check_session)
    rospy.Service('get_score', GetScore, bagger.get_score)

    rospy.Subscriber('tags', Tags, bagger.handle_tags)
    rospy.Subscriber('sensorPacket', SensorPacket, bagger.handle_sensor_packet)
    rospy.Subscriber('cmd_vel', Twist, bagger.handle_cmd_vel)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        bagger.print_to_file()
        rate.sleep()
