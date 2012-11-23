#!/usr/bin/python
import roslib; roslib.load_manifest('ar_localizer')
import rospy
from math import sin, cos, pi
from position_tracker.msg import Position
from position_tracker.srv import SetPosition
from ar_recog.msg import Tag, Tags

class Localizer:

    def __init__(self):
        self.reported_last = True
        self.last_time = 0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

#        self.ok_to_localize = True
        self.tag_xs = dict()
        self.tag_ys = dict()
        self.tag_thetas = dict()
#        self.position_theta = 0.0
#        self.position_x = 0.0
#        self.position_y = 0.0

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
        rospy.sleep(5.0)

    def localize(self, tags):
        best_tag_cf = 0
        for t in tags.tags:
            if t.id in self.tag_xs \
                    and t.cf > best_tag_cf \
                    and t.cf > 0.90 \
                    and t.distance < 2000:
                best_tag_cf = t.cf

                # This gives us x and y relative to the AR tag if it was unrotated at the origin
                rel_x = t.xMetric
                rel_y = t.yMetric

                # Now we rotate x and y so they're relative to the tag's actual rotation
                actual_theta = self.tag_thetas[t.id] + t.yRot
                x = (rel_x*cos(actual_theta)) - (rel_y*sin(actual_theta))
                y = (rel_x*sin(actual_theta)) + (rel_y*cos(actual_theta))

                # And translate back to the universal coordinate system
                x += self.tag_xs[t.id]
                y += self.tag_ys[t.id]
                theta = pi + actual_theta
                if theta > pi:
                    theta -= 2*pi
                elif theta < -pi:
                    theta += 2*pi

        if best_tag_cf > 0:

            self.last_time = tags.header.stamp.to_sec()
            self.last_x = x
            self.last_y = y
            self.last_theta = theta
            self.reported_last = False

    def report(self):
        if self.reported_last:
            return
        rospy.wait_for_service('set_position')
        try:
            set_position = rospy.ServiceProxy('set_position', SetPosition)
            resp = set_position(self.last_x, self.last_y, self.last_theta, self.last_time + 0.2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.reported_last = True

if __name__ == '__main__':
    rospy.init_node('ar_localizer')
    localizer = Localizer()

    #Replace with non-hard-coded tag node, maybe?
    rospy.Subscriber('tags', Tags, localizer.localize)

    rate = rospy.Rate(4) # Test to find how long this actually takes
    while not rospy.is_shutdown():
        localizer.report()
        rate.sleep()

