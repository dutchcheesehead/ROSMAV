#!/usr/bin/python
import roslib; roslib.load_manifest('omdc')
import rospy
import sys
from planar_tracker.msg import Tag_Position, Tag_Positions
from om_msgs.msg import Overhead_Map_Obj, Overhead_Map_Objs

# globals
pub = False

def tpup(tps):
    global pub
    omos = Overhead_Map_Objs()
    omos.objs = []
    # copy all the tags from tag_positions over
    for i in range(len(tps.tag_positions)):
        tp = tps.tag_positions[i]
        omo = Overhead_Map_Obj()
        # set the tuple now because we know we want an image,
        #   since we are getting arTag / blob info. from planar_tracker.
        omo.tuple = [tp.x, tp.y, tp.theta]
        if tp.id < 0: # blob from cmvision; assume it's a ball
            omo.name = "ball"
        else: # arTag; assume it's an iRobot Create
            omo.name = "irobot-create"
        omos.objs.append(omo)
    pub.publish(omos)

def go():
    global pub
    pub = rospy.Publisher('overhead_map_objs', Overhead_Map_Objs)
    rospy.init_node('omdc')
    rospy.Subscriber('tag_positions', Tag_Positions, tpup)
    rospy.spin()

if __name__ == '__main__':
    go()


