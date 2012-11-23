#!/usr/bin/env python
import roslib; roslib.load_manifest('step_nav')
import rospy
from goal import Goal, GoalStack
import math
from geometry_msgs.msg import Twist
from position_guesser.msg import Belief, Point

# This class monitors the quality of the navigation data, and tells
# the robot to whine when it needs to improve things.
def class StepMonitor(object):
    # Initialize with a tuple indicating the number of dimensions
    # we're using and whether any are periodic.
    def __init__(self, dims):
        self.tolerances = [0.1] * len(dims) # Pretty arbitrary.
        self.dims = dims
        self.stds = [0] * len(dims)
        self.means = [0] * len(dims)

    def updateGuess(self, req):
        self.means = req.Points.means
        self.stds = req.Points.stds

        if (sum(map(lambda x, y: x > y, self.stds, self.tolerances)) > 0):
            print "scream"
            # Send brake message to driver and maybe push a new goal
            # onto the stack where we would go for position
            # confirmation.


# This is also where we can monitor the sensor packet for bumps and
# add a perturbation to get around obstacles by pushing a new goal
# onto the stack.



if __name__ == '__main__':
    rospy.init_node('step_mon')
    sm = StepMonitor((0.0, 0.0, 2 * math.pi))

    rospy.Subscriber('guess', Points, sm.updateGuess)
