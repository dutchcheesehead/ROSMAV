#!/usr/bin/env python
#
# This is the controlling logic for a 1-dimensional robot, as in model-1d.
#
import roslib ; roslib.load_manifest('cover')
import rospy
from cover.msg import Room
from cover.srv import MRFQuery, MRFQueryRequest, MRFQueryResponse
from irobot_create_2_1a.msg import SensorPacket
from perfesser.msg import Belief, Pt
from perfesser_guesser import Guesser
from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
import numpy as np
import math



#from perfesser_guesser import Guesser

  # We are now looking at a robot control that implements a service.
  # The service involves accepting a question in the form of "what do
  # you think about my proposed move to X?" with N versions of X.  The
  # reply is a collection of ratings (0-1] for each of the points.  We
  # cache the ratings and ask all our neighbors.  When done, we are
  # equipped both to answer similar questions from those same
  # neighbors, and to incorporate all the answers, resample, and
  # choose a single move out of this.
  # 
  # We can still use the away(), space(), and assorted similar
  # functions to come up with the question (the 'belief'), but the
  # rest can be trashed.
  #
  # We need a class to encapsulate the neighbor relations.  Must have
  # a place to cache the replies to questions.  A controller class
  # that implements the query service call, and also publishes
  # cmd_vel, or specifies a goal to the localization apparatus.  Also
  # a way to discover who our neighbors are.  (In the wild this would
  # be via some kind of direct measurement.)


class Neighbor(object):
    """
    This class stores information about a neighboring robot, including
    the last message it sent to us and its location (presumably
    measured by AR tag or radio signal strength or something).
    """
    def __init__(self, name, xpos=0.0):
        self.name = name
        # This is a tentative position, used in the demo to figure out
        # which robots we can simulate "seeing."
        self.xpos = xpos

        self.lsub = rospy.Subscriber(self.name + "/position", Belief, self.getPos)

        # Use this to hold our measurement of the neighboring robot's
        # position.
        self.location = Belief(data_type = "location",
                               data_sub_type = self.name,
                               pers = (0.0,))
        # Use this to hold the last question sent to the neighboring
        # robot.  This 'question' would have been a suggestion about
        # appropriate targets for that neighboring robot to move
        # towards.
        self.lastQuestion = Belief(data_type = "destination",
                                   data_sub_type = self.name,
                                   pers = (0.0,))
        # This is the reply to the question stored above, a collection
        # of ratings of each of the points in the question above.
        # Each point is a floating-point value x: 0 < x < 1.
        self.lastReply = Belief(data_type = "rating",
                                data_sub_type = "from " + self.name,
                                pers = (0.0,))

        # This is the service by which we send the lastQuestion and
        # receive the lastReply.
        self.advise = rospy.ServiceProxy(self.name + "/advice",
                                         MRFQuery)

        # This is the latest suggestion sent to us by this neighbor.
        self.lastSuggestion = Belief(data_type = "suggestion",
                                   data_sub_type = self.name,
                                   pers = (0.0,))
                   
        # The reply we made to the suggestion above.
        self.rating = Belief(data_type = "rating",
                             data_sub_type = "from host",
                             pers = (0.0,))


    def str(self):
        out = ""
        out += "Name=%s, Location=%.3f\n" % (self.name, self.xpos)
        out += str(self.location)
        out += str(self.lastQuestion)
#        out += str(self.lastReply)
        return out

    def getPos(self, req):
        """
        Receives a Belief object containing the robot's location
        estimate.
        """
        self.location = req


class modelControl(object):
    def __init__(self, name, initpos=0.0, npoints = 70):
        print "Starting." , name
    
        self.name = name
        self.xpos = initpos
        self.npoints = npoints

        # This is a list of neighboring robots, assumed to contain
        # Neighbor() objects, as above.
        self.neighbors = []

        self.velPub = rospy.Publisher(self.name + "/cmd_vel", Twist)
        self.posPub = rospy.Publisher(self.name + "/position", Belief)

        rospy.sleep(1.0)

        self.sensorSub = rospy.Subscriber(self.name + "/sensors", SensorPacket, 
                                          self.trackSensors)

        self.odomSub = rospy.Subscriber(self.name + "/odom", Odometry, 
                                        self.trackOdom)

        self.advice = rospy.Service(self.name + "/advice", MRFQuery, 
                                    self.takeAdvice)

        self.roomLoc = rospy.Subscriber("/room/locations", Room, 
                                        self.recordNeighbors)

        # This is an array of points in configuration space indicating
        # where we need to go next.
        self.deltaConfig = np.array([[]])

        self.obstacles = []

        self.twist = Twist()
        self.twist.linear.x = 0.03

        self.xgoal = 0.0
        self.maxSpeed = 0.15

        # These are coefficients for the opinion formation.  Tune and forget.
        self.aw = 0.05
        self.sp = 0.0
        self.av = 0.0
        self.se = 0.0

        self.maxDist = 0.38
        # Estimated variance in the position measurements.
        self.std = 0.03

        # When rating a neighbor's opinion about our desired position,
        # use this many bins.  Shouldn't be a radically different
        # order of magnitude from npoints.
        self.nbins = 10

        self.busy = False

    def recordNeighbors(self, req):
        """
        At any time, the self.neighbors list should contain a Neighbor
        object for each robot whose position we can measure directly.  

        For the purpose of simulation, we are receiving a list of
        robots and positions via the /room/locations topic and
        accumulating.  Use this as a callback to the /room/locations
        topic.  The result of calling this is an updated
        self.neighbors array.
        """
        distances = []

        for name, loc in zip(req.names, req.locations):
            # Exclude the robot that matches us.
            if name != self.name:
                distances.append([math.fabs(self.xpos - loc.point[0]), name])

        # Now we have a list, sortable by the distance between us and
        # the robot.  Take the top two and see if they are in
        # self.neighbors.  If not, add one.  Remove from
        # self.neighbors anything that isn't in this list.
        distances.sort()

        # Is this robot at one end or the other of the group?
        atEnd = (name == req.names[0]) or (name == req.names[-1])
#        atEnd = False

        toBeDeleted = []
        for nb in self.neighbors:
            if nb.name == distances[0][1]:
                distances[0][1] = ""
            elif (not atEnd) and nb.name == distances[1][1]:
                distances[1][1] = ""
            #elif nb.name == distances[2][1]:
            #    distances[2][1] = ""
            else:
                # Got one that needs deleting from self.neighbors.
                toBeDeleted.append(nb)

        if distances[0][1]:
            self.neighbors.append(Neighbor(name=distances[0][1]))
            #print self.name, " taking...1.....", distances[0][1]
        if (not atEnd) and distances[1][1]:
            self.neighbors.append(Neighbor(name=distances[1][1]))
            #print self.name, " taking...2.....", distances[1][1]
        #if distances[2][1]:
        #    self.neighbors.append(Neighbor(name=distances[2][1]))
        if toBeDeleted:
            for nb in toBeDeleted:
                self.neighbors.remove(nb)

    def trackSensors(self, sensors):
        if sensors.bumpLeft or sensors.bumpRight:
            if not self.xpos in self.obstacles:
                self.obstacles.append(self.xpos)

    def getPosEst(self):
        out = Belief(source_stamp = rospy.Time.now(),
                     source_data = "estimate",
                     data_type = "location",
                     data_sub_type = self.name,
                     sender = self.name,
                     dim_names = [ self.name + "/xpos" ],
                     pers = (0.0,),
                     means = [self.xpos],
                     stds = [ self.std ])

        for x in np.random.normal(loc = self.xpos, scale = self.std, 
                                  size = self.npoints):
            out.points.append(Pt(point = (x,)))
        
        return out

    def trackOdom(self, req):
        # This gets called as often as a new odom is issued.

        self.xpos = req.pose.pose.position.x

#        print "trackOdom: ", self.name

        self.posPub.publish(self.getPosEst())

        self.evaluateMove()

        # out = []
        # print "Asking (%d) neighbors of: %s" % (len(self.neighbors), self.name)
            # print  ("Asker: %s, asked: %s" % (m.asker, m.asked)), reply.reply
            # out.append(reply.reply)

        # print "Replies of: ", self.name, out


    def takeAdvice(self, req):
        """
        Takes a piece of advice from its neighbor, rates it, and
        returns the rating.  The advice is a distribution of points to
        which the other robot thinks this robot ought to go.  We
        compare that advice to our own intentions and return ratings
        indicating the degree of agreement between the two: 1.0 is
        complete agreement and 0.0 is complete disagreement.  We never
        return exactly 0.0 as a matter of policy.
        """
        if not req.question:
            return MRFQueryResponse(no_data = True)
        if not self.deltaConfig.any():
            return MRFQueryResponse(no_data = True)

        # Allocate our intention into bins.
        question = self.ptsToArray(req.question)

        # Establish bins so that there is one point in each outer bin.
        bins = np.linspace(min(self.deltaConfig[:,0]) + 1.e-10,
                           max(self.deltaConfig[:,0]) - 1.e-10, self.nbins)
        hist = [ 0.0 ] * (1 + len(bins))

        # Create a distribution of points that includes neighbor's
        # opinions except for the one asking the question.
        phi = self.deltaConfig[:,0]
        
        messageProduct = [ 1.0 ] * len(phi)
        for nb in self.neighbors:
            if (nb.name != req.asker) and (nb.lastReply.points):
                messageProduct = [ m * p.point[0] for m,p in \
                                   zip(messageProduct,
                                       nb.lastReply.points) ]

        phi = self.resample(phi, messageProduct)

        # Populate bins
        for delta in phi:
            hist[sum(delta > bins)] += 1.0

        n = sum(hist)

        # Formulate a reply by binning the input data.
        m = MRFQueryResponse(no_data = False,
                             source_stamp = req.source_stamp)
        for q in req.question:
            m.reply.append(Pt(point=( hist[sum(q.point[0] > bins)]/n, )))

        # print "entering service: ", self.name
        # print self.name, "'s opinion: ", self.deltaConfig
        # print "question: ",question
        # print "hist: ", hist
        # print "bins: ", bins
        # print m
        return m

    def ptsToArray(self, pts):
        return np.array([p.point for p in pts])

    def arrayToPts(self, array):
        return [ Pt(point = a) for a in array ]

    def setAdvice(self, name, advice):
        """
        Takes a name of a robot and an array of advice and stores it
        in the neighbor array -- if appropriate.
        """
        for nb in self.neighbors:
            if nb.name == name:
                nb.lastQuestion.points = self.arrayToPts(advice)

    def evaluateMove(self):
        """
        Formulates a distribution of points representing a belief
        about the direction in which we should move, along with our
        neighbors.  The belief is based on our measurements of our own
        location, as well as the location of the other robots, along
        with some goals (i.e. spread out, explore, etc).

        To be explicit, inputs are these:
          - Location of self
          - Location of neighbors
          - Suggestions from neighbors about where self should move. 
          - Replies by neighbors to suggestions about where they
        should move.

        The first two form the input to the phi() function.  The third
        is the input to the psi() function, and the fourth is the
        messageProduct referenced below.

        Definition: "configuration space" is the n-dimensional space
        where each point represents a particular configuration of all
        the robots in it.  So a one-dimensional room containing three
        robots corresponds to a three-dimensional configuration
        space.  A two-dimensional room with five robots makes ten
        dimensions in configuration space.  Seems confusing, but it
        makes some of the math easier.
        """
        if self.busy:
            return
        self.busy = True

        # Marshal the inputs: our position and the positions of our
        # neighbors.  Park them all in an array where each row
        # represents a position in configuration space and the
        # collection of rows represent the distribution of those
        # points.
        currentConfig = np.array(self.ptsToArray(self.getPosEst().points))

        # For 2d this should be two entries with X and Y
        self.deltaNames = [ self.name ] 

        for nb in self.neighbors:
            if nb.location.points:
                currentConfig = np.hstack([currentConfig, 
                                           self.ptsToArray(nb.location.points)])

                self.deltaNames.append(nb.name)

        # Run through the points in the current configuration and
        # generate predictions for each point in the distribution.
        deltaPts = []
        for cpoint in currentConfig:
            deltaPts.append(self.phi(cpoint))

        self.deltaConfig = np.array(deltaPts)


        # Break up the array, and store the opinions about each
        # neighbor in the neighbor list.  We do it this way in case
        # the neighbor list has changed since the marshaling.
        for nm,dc in zip(self.deltaNames,np.hsplit(self.deltaConfig,
                                         self.deltaConfig.shape[1])):
            self.setAdvice(nm, dc)

        # Notify each neighbor of our opinion (and record their
        # responses).
        messageProduct = [ 1.0 ] * self.npoints
        for nb in self.neighbors:

            # If we don't already have an opinion about where this
            # robot should be then pass for the moment and we'll get
            # it the next time around.
            if not nb.lastQuestion.points:
                break

            m = MRFQueryRequest(source_stamp = rospy.Time.now(),
                                asker = self.name,
                                asked = nb.name,
                                question = nb.lastQuestion.points)
            r = nb.advise(m)
            if r.reply:
                nb.lastReply.points = r.reply
                nb.lastReply.source_stamp = r.source_stamp
                messageProduct = [ lr.point[0] * mp \
                                       for lr, mp in zip(r.reply,
                                                         messageProduct) ]

        # Resample based on the neighbor's responses.
        desiredConfig = self.resample(self.deltaConfig, messageProduct)

        # The first set of coordinates in this is the desired place
        # for us to move.
        move = desiredConfig[:,0]

        # Issue a command to move somewhere.
        limit = lambda(x): math.copysign(min(self.maxSpeed, math.fabs(x)),x)

        self.twist.linear.x = limit(np.mean(move))# - self.xpos)
        self.velPub.publish(self.twist)

        self.busy = False

    def weightedSample(self, pointProbs, numSamples):
        """
        The following sampling algorithm is adapted from a sneaky
        algorithm for coming up with a selection of N random numbers
        between 0 and Q.  The idea is that the probability that all N
        of the numbers are less than some X is P=(X/Q)**N.  Solve for
        X in terms of P, and you get an equation that will pick the
        largest number in that distribution for some random P.  Do
        that N times, and you get N samples.
    
        The input is an indexed array of probabilities (use
        enumerate(P) for the input, e.g. and the number of samples to
        take from that array.  The output is an iterator that will
        provide the indices you can use to sample the original array
        with, as in:
    
        output = locArray[weightedSample(enumerate(locProbs), n)

        This won't exactly work, since the enumerate has to be turned
        into a sequence before it can be subscripted (as below) and
        the same is true of the function output.

        For more: stackoverflow.com/questions/2140787
        """

        total = sum(prob for i, prob in pointProbs)
        assert(not math.isnan(total))
        j = 0
        i, prob = pointProbs[j]
        while numSamples:
            x = total * (1 - np.random.random() ** (1.0/numSamples))
            total -= x
            while x > prob:
                x -= prob
                j += 1
                i, prob = pointProbs[j]
            prob -= x
            yield i
            numSamples -= 1

    def resample(self, array, ratings):
        """
        Resamples the distribution represented by the samples in
        array, based on the list of ratings.  There should be one
        rating for each row of the array, and each rating should be
        greater than zero and less than or equal to 1.0.  The ratings
        list need not be normed.
        """
        #print "resample according to: ", ratings

        newseq = self.weightedSample([(i,p) for i,p in enumerate(ratings)], 
                                     len(ratings))
        #print "comes up with: ", [p for p in newseq]

        wiggle = lambda x: self.std * (np.random.random() - 0.5) + x

        return array[[ wiggle(p) for p in newseq],:]


    def away(self, cpoint):
        """
        Calculates the efficient vector moving away from the identity
        line.  (3-d).  This is derived by finding the parameterized
        description of the line that goes through the input point and
        the closest point to it on the identity line.  You wind up
        with n equations of the form x = xzero + (a - xzero) * t where
        a is the input x coordinate and xzero is the nearest point on
        the identity line.  Set t = 1.1, for no particular reason.
        """
        if len(cpoint) < 2:
            return ( 0.0, )

        # Find centroid
        xzero = sum(cpoint)/len(cpoint)

        # Parameterize travel from centroid, set t = 1.1
        vec = [ (x * 0.1) - (xzero * 0.1) for x in cpoint ]

        # Normalize
        sumsq = (sum([ v**2 for v in vec ]))**0.5
        if sumsq > 0:
            return tuple( [ v/sumsq  for v in vec ])
        else:
            return vec

    def dist(self, x): 
        """
        Find and rank the distances between pairs from a set of points.
        """
        out = []
        for i in range(len(x)):
            for j in range(i + 1,len(x)):
                out.append(math.fabs(x[i] - x[j]))
        out.sort()
        out.reverse()
        return out       

    def space(self, cpoint, away):
        return tuple([ a for a in cpoint ])

    def avoid(self, cpoint):
        return tuple([ a for a in cpoint ])

    def seek(self, cpoint):
        return tuple([ a for a in cpoint ])

    def phi(self, cpoint):
        """
        Using the input cpoint of points in configuration space,
        formulate a suggestion about where we would like to move and
        where we think the others in this cpoint should move, too.  The
        first column (or columns) is assumed to correspond to ourself,
        the bookkeeping for the other columns is left to the caller.
        """
        ## Find the four primary 'directions' in configuration space.
        away = self.away(cpoint)
        space = self.space(cpoint, away)
        avoid = self.avoid(cpoint)
        seek = self.seek(cpoint)

        ## Find distances between all the points.
        dists = self.dist(cpoint)
        #print cpoint, dists

        if dists:
            if len(dists) > 2:
                d = dists[1]
            else:
                d = dists[0]

            if d > self.maxDist:
                away = [ -a for a in away ]

        
#        print self.name,">>", [self.aw * w + self.sp * x + self.av * y + self.se * z \
#                                   for w,x,y,z in zip(away, space, avoid, seek) ]


        # Add them.
        return [self.aw * w + self.sp * x + self.av * y + self.se * z \
                    for w,x,y,z in zip(away, space, avoid, seek) ]


    def psi(self, advice):
        """
        Compatibility function.  Given a suggestion in the 'advice'
        variable, this is the collection of advice messages given by
        our neighbors (or at least the neighbors who are paying
        attention).
        """
        pass


    def rate(self, advice):
        pass

if __name__ == "__main__":

    rospy.init_node("cover")

    print "ENTERING model_1d_control"

 # Launch with appropriate name and pubs and subs

    currentRobotNameList = []
    currentRobotList = []
    def reviewNumberOfRobots(req):

        # If we don't have a controller for a robot, add one.
        for name in req.names:
            if not (name in currentRobotNameList):
                currentRobotList.append(modelControl(name))
                currentRobotNameList.append(name)
                print "CREATING ----> ", name

        # If we have a controller for a robot that isn't there, delete
        # it. 
        for name in currentRobotNameList:
            if not (name in req.names):
                currentRobotNameList.remove(name)
                for robot in currentRobotList:
                    if robot.name == name:
                        currentRobotList.remove(robot)

    
    locsub = rospy.Subscriber("/room/locations", Room, 
                              reviewNumberOfRobots)



#    abel = modelControl("abel")
#    baker = modelControl("baker")
#    charlie = modelControl("charlie")
#    dog = modelControl("dog")
#    easy = modelControl("easy")

    rospy.spin()


    # r = Room(names = ["abel", "baker", "charlie", "dog", "easy", "fox"],
    #          locations = [0.15, 0.25, 0.35, 0.45, 0.55, 0.65])

    # c = modelControl("charlie")
    # c.xpos = 0.41

    # c.neighbors.append(Neighbor(name="abel", xpos=0.22))
    # c.neighbors.append(Neighbor(name="baker", xpos=0.32))


    # for nb in c.neighbors:
    #     print "<<<", nb.str()

    # c.recordNeighbors(r)

    # for nb in c.neighbors:
    #     print ">>>", nb.str()
