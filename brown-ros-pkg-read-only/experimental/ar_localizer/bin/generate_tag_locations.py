#!/usr/bin/python
import roslib; roslib.load_manifest('ar_localizer')
import rospy
from math import sin, cos, pi
from position_tracker.msg import Position
from ar_recog.msg import Tag, Tags
from ar_localizer.srv import *

class Localizer:

    def __init__(self):
        self.observation_in_progress = False
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.last_tags = Tags()

        self.tag_xs = dict()
        self.tag_ys = dict()
        self.tag_thetas = dict()

        f = open('tag_locations.txt', 'r')
        for line in f:
            if line.startswith("#"):
                continue
            tag_info = line.split()
            if len(tag_info) != 4:
                continue
            self.tag_xs[int(tag_info[0])] = float(tag_info[1])
            self.tag_ys[int(tag_info[0])] = float(tag_info[2])
            self.tag_thetas[int(tag_info[0])] = float(tag_info[3])
        f.close()

    def trackTags(self, tags):
        if not self.observation_in_progress:
            self.last_tags = tags

    def trackPosition(self, position):
        if not self.observation_in_progress:
            self.last_x = position.x
            self.last_y = position.y
            self.last_theta = position.theta

    def generateLocations(self, req):
        self.observation_in_progress = True
        for t in self.last_tags.tags:

            offset_x = (t.xMetric*cos(self.last_theta)) - \
                (t.yMetric*sin(self.last_theta))
            offset_y = (t.xMetric*sin(self.last_theta)) + \
                (t.yMetric*cos(self.last_theta))
            self.tag_xs[t.id] = self.last_x + offset_x
            self.tag_ys[t.id] = self.last_y + offset_y
            self.tag_thetas[t.id] = self.last_theta + pi + t.yRot
            if self.tag_thetas[t.id] > pi:
                self.tag_thetas[t.id] -= 2*pi
            elif self.tag_thetas[t.id] < -pi:
                self.tag_thetas[t.id] += 2*pi

        f = open('tag_locations.txt', 'w')
        for key in iter(self.tag_xs):
            towrite = str(key) + '\t' + str(self.tag_xs[key]) + '\t' + str(self.tag_ys[key]) + '\t' + str(self.tag_thetas[key]) + '\n'
            f.write(towrite)
        f.close()
        self.observation_in_progress = False
        return GenerateLocationsResponse(True)

if __name__ == '__main__':
    rospy.init_node('generate_tag_locations')
    localizer = Localizer()

    rospy.Subscriber('tags', Tags, localizer.trackTags)
    rospy.Subscriber('position', Position, localizer.trackPosition)
    rospy.Service('generate_locations', GenerateLocations, localizer.generateLocations)

    rospy.spin()
