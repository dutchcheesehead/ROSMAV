#!/usr/bin/env python
#
# Listens on the position_guesser topic and republishes it as position_summary,
# without all the points, and only the means and stds.  Use for debugging.  A
# node interested in only the means should just subscribe to the position_guesser
# topic and only pay attention to the means and stds it will find there.
#
# This exists because rostopic echo --noarr leaves out too much.
#
import roslib ; roslib.load_manifest('perfesser')
import rospy
import numpy as np
import math
import tgraph
from perfesser.msg import Belief, Pt

# Accepts a Belief message and republishes a message containing only
# the means and stds.
class Summarizer(object):
    def __init__(self, pub, graph=False):
        self.pub = pub
        self.graph = graph
        if self.graph:
            self.tg = tgraph.Tgraph(300,225)
            parray = np.array([[-3.0,-3.0, -math.pi],[3.0,3.0,math.pi]])
            self.tg.draw_scatter(parray, 0,1,2, "s")
            self.tg.clearObjects()
            print "*********************************************"
        self.x = 0.0

    def summarize(self, msg):
        msg.points = []
        self.pub.publish(msg)

        if False: #True: #self.graph:
#            self.tg.clearObjects()
            print "next>>>>>>>>>>>>>>>>>>>>", self.graph, len(msg.points)
            plist = []
#            for p in msg.points:
#                plist.append(p.point)
            plist.append([self.x, self.x, 0.0])
            self.tg.plot_points(np.array(plist), 0,1,2, "s")
            self.x += 0.05

if __name__ == '__main__':
    rospy.init_node('perfesser_summarizer')
    print "init done"

    pub = rospy.Publisher('/guesser/position/summary', Belief)
    print "publisher defined"

    s = Summarizer(pub, graph=False)
    print "summarizer created"

    rospy.Subscriber('/guesser/position/guess', Belief, s.summarize)
    print "subscriber nominated"

    bpub = rospy.Publisher('/guesser/bumps/summary', Belief)
    print "bump publisher defined"

    b = Summarizer(bpub, graph=False)
    print "bump summarizer created"

    rospy.Subscriber('/guesser/bumps/obstacles', Belief, b.summarize)
    print "subscriber nominated"

    rospy.sleep(5.0) # wait for subscriber to get set up
    print "ready, go"

    rospy.spin()

