#!/usr/bin/python
###############################################################################
# Multicamera Resolution
###############
# Brian Thomas (brianjaythomas@gmail.com)
# Input: Subscribes to topics of type Tag_Positions.  These topics are
#   provided as command-line arguments, and any number >= 1 can be input.
# Output: Publishes tag_positions topic of type Tag_Positions.  This topic
#   is the result of merging the inputs in the following way:  For each ARtag
#   of a given id, average the values reported for this id over all cameras.
#   (It is assumed that each object seen by the set of cameras has a unique
#   ARtag.)
###############################################################################
import roslib; roslib.load_manifest('planar_tracker')
import rospy
import sys
from copy import deepcopy
from time import clock
from math import sin,cos,atan2
from planar_tracker.msg import Tag_Position, Tag_Positions

class TPSubscription:
    name = ""
    positions = []
    def __init__(self,name):
        self.name = name
    def updatePositions(self,x):
        self.positions = x
    def getPositions(self):
        return self.positions

def go():
    pub = rospy.Publisher('tag_positions', Tag_Positions)
    rospy.init_node('multicamera_resolution')
    args = rospy.myargv(sys.argv)
    # Check for erronous output
    if (len(args) == 1 or
        args[1] == "-h" or
        args[1] == "-help" or
        args[1] == "--help"):
        rospy.logerr("Usage: "+args[0]+" [Tag_Positions topic]+")
        sys.exit(2)
    # Each subscription is scheduled to update tpsubscriptions on execution
    tpsubscriptions = map(lambda x: [], range(len(args[1:])))
    for i in range(len(args[1:])):
        subscriberString = args[i+1]
        rospy.loginfo("Subscribing to " + subscriberString);
        tpsubscriptions[i] = TPSubscription(subscriberString)
        rospy.Subscriber(subscriberString,
                         Tag_Positions,
                         # outer lambda required for closure
                         (lambda i: lambda x: tpsubscriptions[i].updatePositions(x.tag_positions))(i))
    # Broadcast currently-known position at FREQUENCY
    FREQUENCY = 30 # hz
    while not rospy.is_shutdown():
        startTime = clock()
        # try to avoid tpsubscriptions as much as possible, because it's
        #   constantly updating
        subs = deepcopy(tpsubscriptions)
        # group by arTag id.  Assume each tag appears on no more than one object
        artagsTPositions = {}
        for sub in subs:
            for position in sub.getPositions():
                if position.id in artagsTPositions:
                    artagsTPositions[position.id].append(position)
                else:
                    artagsTPositions[position.id] = [position]
        artagsTPositions = artagsTPositions.values()
        # Resolve by averaging; publish a Tag_Positions topic.
        arTags = map(arTagPositionResolution, artagsTPositions)
        tps = Tag_Positions()
        tps.tag_positions = arTags
        pub.publish(tps)
        # sleep until startTime + 1./FREQUENCY time
        rospy.sleep(1./FREQUENCY - (startTime - clock()));

def arTagPositionResolution(artagsTPositions):
    # averages several AR Tag position reports into one
    count = len(artagsTPositions)
    position = Tag_Position()
    position.id = artagsTPositions[0].id
    position.x = 1.*sum(map(lambda(atp): atp.x, artagsTPositions))/count
    position.y = 1.*sum(map(lambda(atp): atp.y, artagsTPositions))/count
    # break each theta into its vector representation, and use
    #   atan2 to reconstruct an "average" theta
    thetaCosSum = 1.*sum(map(lambda(atp): cos(atp.theta), artagsTPositions))
    thetaSinSum = 1.*sum(map(lambda(atp): sin(atp.theta), artagsTPositions))
    position.theta = atan2(thetaSinSum, thetaCosSum) # (y,x)
    return position

if __name__ == '__main__':
    go()

