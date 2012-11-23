#!/usr/bin/python
#
# Planar tracker node
# This node uses a perspective transform to calculate cartesian coordinates of the object
# given its coordinates in the image plane
# Also listens to /blobs topic where blobs from the blobfinder are published. Blobs are
# converted into tags and perspective transofrm is applied to them as well. Resulting
# information on blobs and tags is published to the /tag_positions topic (topic name can be
# specified via the command line).
#
import roslib; roslib.load_manifest('planar_tracker')
import rospy
import numpy
import cv
import math

from math import pi
from ar_recog.msg import Tag, Tags
from planar_tracker.msg import Tag_Position, Tag_Positions
from cmvision.msg import Blob, Blobs

#Name of the topic where tag/ball positions will be published
topic_name = 'tag_positions'

#
# Ball tracking
#
#Minimum screen area of the ball blob so it will be reported.
#Blobs with area less than this will be ignoder
area_crit = 100

#ID for the ball
ball_id = -1

#
# Correction stuff
# If there is a sistematic bias in one of the coordinates, it is possible to correct
# it. See correct() function for details
#
#Field dimensions
maxX = 4.0
maxY = 2.4
#Step for corrections array
step = 0.4

arr_correct_theta=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

arr_ball_pos = []

#Make corrections for coordinates (x,y) and array arr_correct
#In fact, this is 4-NN algorithm
def correct(x, y, arr_correct):
	ix_hi=0
	iy_hi=0
	ix_lo=0
	iy_lo=0

	#Efficient X,Y coordinates. They will be used to find correction points in the array
	x_eff = x
	y_eff = y

	#Cut everything that is off the field
	if (x_eff <= 0):
		x_eff = 0.01
	if (y_eff <= 0):
		y_eff = 0.01
	if (x_eff >= maxX):
		x_eff = maxX-0.01
	if (y_eff >= maxY):
		y_eff = maxY-0.01

	#Find 4 adjacent cells
	ix_hi = int(x_eff/step)+1
	ix_lo = int(x_eff/step)
	iy_hi = int(y_eff/step)+1
	iy_lo = int(y_eff/step)

	#Find distance between (X,Y) and cell centers
	dx = math.fmod(x_eff, step)
	dy = math.fmod(y_eff, step)
	if (dx < 0.01):
		dx = 0.01
	if (dy < 0.01):
		dy = 0.01

	#Square distances to speed up computations
	dx2 = dx*dx
	dy2 = dy*dy
	ndx2 = (step-dx)*(step-dx)
	ndy2 = (step-dy)*(step-dy)

	sumCorrect = 0
	sumdt = 0

	#Calculate contribution from every cell. The bigger is distance,
	#the smaller is contribution

	dt = (dx2+dy2)	#distance from [x_lo, y_lo]
	correct = arr_correct[iy_lo][ix_lo]/dt;
	sumCorrect = sumCorrect + correct
	sumdt += 1/dt;

	dt = (ndx2+dy2)	#distance from [x_hi, y_lo]
	correct = arr_correct[iy_lo][ix_hi]/dt;
	sumdt += 1/dt;
	sumCorrect = sumCorrect + correct

	dt = (dx2+ndy2)	#distance from [x_lo, y_hi]
	correct = arr_correct[iy_hi][ix_lo]/dt;
	sumdt += 1/dt;
	sumCorrect = sumCorrect + correct

	dt = (ndx2+ndy2)	#distance from [x_hi, y_hi]
	correct = arr_correct[iy_hi][ix_hi]/dt;
	sumdt += 1/dt;
	sumCorrect = sumCorrect + correct

	#Normalize the result
	return sumCorrect / sumdt;


class Tracker:

    def __init__(self):
        self.transform = cv.CreateMat(3, 3, cv.CV_64FC1)
        f = open('transformation_points.txt', 'r')
        image_plane = []
        object_plane = []
        count = 0
        zRot_total = 0.0
        for line in f:
            if line.startswith("#"):
                continue
            transforms = line.split()
            if len(transforms) != 5:
                continue
            image_plane.append((float(transforms[0]), float(transforms[1])))
            object_plane.append((float(transforms[2]), float(transforms[3])))
            zRot_total += float(transforms[4])
            count += 1
        cv.GetPerspectiveTransform(image_plane, object_plane, self.transform)
        self.zRot_offset = zRot_total / count

        self.pub = rospy.Publisher(topic_name, Tag_Positions)

    def track_ball(self, blobs):
	global arr_ball_pos
        arr_ball_pos = []
        tag_positions = Tag_Positions()
        for blob in blobs.blobs:
            if (blob.red == 255 and blob.green == 0 and blob.blue == 0 and blob.area > area_crit):
                current_tag_position = Tag_Position()
                current_tag_position.id = ball_id
                a = cv.fromarray(numpy.array([[[float(blob.x), float(blob.y)]]]))
                b = cv.fromarray(numpy.empty((1,1,2)))
                cv.PerspectiveTransform(a, b, self.transform)
                current_tag_position.x = numpy.asarray(b)[0,0,0]
                current_tag_position.y = numpy.asarray(b)[0,0,1]
	        arr_ball_pos.append(current_tag_position)


    def localize(self, tags):
        global arr_ball_pos
        tag_positions = Tag_Positions()
        for t in tags.tags:
            current_tag_position = Tag_Position()
            current_tag_position.id = t.id

            #This is all Open CV matrix stuff that is more complicated
            #than it should be.
            a = cv.fromarray(numpy.array([[[float(t.x), float(t.y)]]]))
            b = cv.fromarray(numpy.empty((1,1,2)))
            cv.PerspectiveTransform(a, b, self.transform)
            current_tag_position.x = numpy.asarray(b)[0,0,0]
            current_tag_position.y = numpy.asarray(b)[0,0,1]

            current_tag_position.theta = t.zRot - self.zRot_offset

            #Adjust tag angle based on its position
            current_tag_position.theta += correct(current_tag_position.x, current_tag_position.y, arr_correct_theta)

            if (current_tag_position.theta < -pi):
                current_tag_position.theta += 2*pi
            if (current_tag_position.theta > pi):
                current_tag_position.theta -= 2*pi

            tag_positions.tag_positions.append(current_tag_position)
        for tcache in arr_ball_pos:
            tag_positions.tag_positions.append(tcache)

        self.pub.publish(tag_positions)


if __name__ == '__main__':

    argv = rospy.myargv()
    if len(argv) == 2:
        topic_name = argv[1];

    rospy.init_node('planar_tracker')
    tracker = Tracker()

    rospy.Subscriber('tags', Tags, tracker.localize)
    rospy.Subscriber('blobs', Blobs, tracker.track_ball)

    rospy.spin()

