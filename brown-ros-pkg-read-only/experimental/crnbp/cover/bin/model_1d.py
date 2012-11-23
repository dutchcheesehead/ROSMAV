#!/usr/bin/env python
# Uses the model_create stuff to model a 1-d room with three robots in it.
#
import roslib ; roslib.load_manifest('cover')
import rospy
from geometry_msgs.msg import Twist, Quaternion, Pose
from perfesser.msg import Pt
from nav_msgs.msg import Odometry
from irobot_create_2_1a.msg import SensorPacket
from model_create import CreateModel, RoomModel
from cover.msg import Room
import tgraph as tg

class OneDRobotModel(CreateModel):
    def __init__(self, name, xpos, xvel, color):
        self.name = name
        self.xpos = xpos
        self.xvel = xvel
        self.color = color

        CreateModel.__init__(self, self.xpos, 0.0, 0.0, name=self.name,
                             color=self.color, trails=False, 
                             nose=False, debug=False)

        topic = (self.name + "/") if self.name else ""
        topic += "cmd_vel"

        self.sub = rospy.Subscriber(topic, Twist, self.setVel)

    def setX(self, newX):
        self.x = newX

    def setVel(self, tw):
        tw.angular.z = 0.0
        tw.angular.x = 0.0
        tw.angular.y = 0.0
        self.twist(tw)

    # Advances the robot's position by time dt. But asserts limits.
    def advance(self, dt):
        now = self.then + dt
        elapsed = now - self.then
        self.then = now
        if self.debug:
            print "advancing to t=%5.3f" % (self.then)

        ldist = dt * self.lwSpeed
        rdist = dt * self.rwSpeed

        x = (ldist + rdist)/2.0

        dx = x / dt

        self.x += x

        if self.x >= 1.0:
            self.x = 1.0
            self.sensorPub.publish(SensorPacket(bumpLeft = True))
        elif self.x <= 0.0:
            self.x = 0.0
            self.sensorPub.publish(SensorPacket(bumpRight = True))
 
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = 0
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion

        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = 0

        self.odomPub.publish(odom)



class OneDRoomModel(RoomModel):
    def __init__(self, dt, xdim):
        self.dt =  dt
        self.xdim = xdim

        RoomModel.__init__(self, self.dt, self.xdim, self.xdim/4, 
                           xscale = 1.0, yscale = 1.0, 
                           xmin = -0.25, ymin = -0.5)

    def nearest(self):
        """
        Returns a list of robots in the room, sorted from low xpos to high.
        """
        robotLocs = []
        for rb in self.robots:
            robotLocs.append([rb.x, rb.name])

        robotLocs.sort()

        r = Room(locations = [Pt(point=(loc,)) for loc,name in robotLocs],
                 names = [name for loc,name in robotLocs])

        return r

import math
import numpy as np

def away(x):
    """
    Calculates the efficient vector moving away from the identity line.
    (3-d).  This is derived by finding the parameterized description of
    the line that goes through the input point and the closest point to 
    it on the identity line.  You wind up with n equations of the form 
    x = xzero + (a - xzero) * t where a is the input x coordinate and 
    xzero is the nearest point on the identity line.  Set t = 1.1, for
    no particular reason.
    """
    xzero = sum(x)/len(x)
    return [ (i * 0.1) - (xzero * 0.1) for i in x ]

    
def cross_product(vecs):
    """
    Compute the n-dimensional cross product for a list of n-1
    n-dimensional vectors using the determinant method:

    a11     a12     a13     ... a1n
    a21     a22     a23     ... a2n
    ...
    a(n-1)1 a(n-1)2 a(n-1)3 ... a(n-1)n

    an1 = -det(square matrix created by excluding first column)
    an2 = det(square matrix created by excluding second column)
    an3 = -det(square matrix created by excluding third column)
    an4 = det(square matrix created by excluding fourth column)
    ...

    """
    ndim = len(vecs) + 1
    assert len(vecs[0]) == ndim

    arr = np.array(vecs)

    out = []
    for i in range(ndim):
        dims = range(ndim)
        dims.remove(i)
        val = np.linalg.det(np.take(arr, dims, 1))
        val = -val if (i % 2) == 1 else val
        out.append(val)

    return out  

def minDist(x):
    """
    Find the minimum distance between two of a set of points.
    """
    out = []
    for i in range(len(x)):
        for j in range(i + 1,len(x)):
            out.append(math.fabs(x[i] - x[j]))
    return min(out)

def space(x, b):
    """
    Finds a unit vector in the direction of the biggest component, and 
    returns the cross product of that vector and the away() vector.
    The theory is that this will be a vector that produces no
    significant motion towards or away from the identity line, and
    moves the various robots toward a somewhat more uniform spacing.
    """

    i = 0; jmax = 0; jmin = 0; xmax = -1.e35 ; xmin = 1.e35

    for e in x:
        if e > xmax:
            jmax = i
            xmax = e
        if e < xmin:
            jmin = i
            xmin = e
        i += 1

    f = [ 0 ] * len(b)
    f[jmax] = 1.0
#    f[jmin] = 1.0

    print "move: (%.3f,%.3f,%.3f)" % tuple(f)

    tentative = cross_product([b,f])

    origMin = minDist(x)

    if minDist([ y+g for y,g in zip(x,tentative)]) < origMin:
        out = [ -y for y in tentative ]
    else:
        out = tentative

    print "prod: (%.3f,%.3f,%.3f)" % tuple(out)

    # 3-d cross product
    return out



if __name__ == '__main__':
    rospy.init_node('cover')
    import time
    import signal
    import sys
    import traceback

    def handler(signum, frame):
        print "....all right already, I'm stopping.  Jeez what a nag."
        raise

    signal.signal(signal.SIGINT, handler)

    room = OneDRoomModel(0.1, 1000)

    robots = []
    def addRobot(name, location, color):
        robots.append(OneDRobotModel(name, location, 0.0, color))
        room.addRobot(robots[-1])

    # abel = OneDRobotModel("abel", 0.75, 0.0, 0.0)
    # baker = OneDRobotModel("baker", 0.7, 0.0, 25.0)
    # charlie = OneDRobotModel("charlie", 0.65, 0.0, 50.0)
    # dog = OneDRobotModel("dog", 0.5, 0.0, 75.0)
    # easy = OneDRobotModel("easy", 0.45, 0.0, 100.0)
    
    # room.addRobot(abel)
    # room.addRobot(baker)
    # room.addRobot(charlie)
    # room.addRobot(dog)
    # room.addRobot(easy)

    # rospy.Subscriber("/abel/cmd_vel", Twist, abel.setVel)
    # rospy.Subscriber("/baker/cmd_vel", Twist, baker.setVel)
    # rospy.Subscriber("/charlie/cmd_vel", Twist, charlie.setVel)
    # rospy.Subscriber("/dog/cmd_vel", Twist, charlie.setVel)
    # rospy.Subscriber("/easy/cmd_vel", Twist, charlie.setVel)

    addRobot("abel", 0.75, 0.0)
    addRobot("baker", 0.72, 20.0)
    addRobot("charlie", 0.7, 40.0)
    addRobot("dog", 0.45, 60.0)
    addRobot("easy", 0.42, 80.0)


    locpub = rospy.Publisher("/room/locations", Room)

    try:

        start = rospy.Time.now().to_sec()
        startg = 30.0
        starth = 35.0
        starti = 50.0
        startj = 90.0


        r = rospy.Rate(4)
        while True:
            room.updateRoom()
            if start + 15.0 < rospy.Time.now().to_sec():
                room.delRobot("dog")

            if start + startg < rospy.Time.now().to_sec():
                addRobot("george", 0.5, 95.0)
                startg = 10000000

            if start + starth < rospy.Time.now().to_sec():
                addRobot("how", 0.2, 100.0)
                starth = 10000000

            if start + starti < rospy.Time.now().to_sec():
                addRobot("ivy", 0.8, 100.0)
                addRobot("joseph", 0.81, 100.0)
                addRobot("kappa", 0.82, 100.0)
                starti = 10000000

            if start + startj < rospy.Time.now().to_sec():
                room.delRobot("george")
                room.delRobot("how")
                room.delRobot("ivy")
                room.delRobot("joseph")
                startj = 10000000

            locpub.publish(room.nearest())
            r.sleep()

    except:
        print "*************EXIT*************"
        print sys.exc_type
        print sys.exc_value
        traceback.print_tb(sys.exc_traceback)
