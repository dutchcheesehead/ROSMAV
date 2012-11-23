#!/usr/bin/python
import roslib; roslib.load_manifest('planar_tracker')
import rospy
import numpy
import cv
import math

#alexta
import sys

from math import pi
from ar_recog.msg import Tag, Tags
#from planar_tracker.msg import Tag_Position, Tag_Positions
from cmvision.msg import Blob, Blobs

#Name of the topic where tag/ball positions will be published
topic_name = 'tags_blobs'

#
# Blob tracking and filtering
#
#Minimum screen area of the ball blob so it will be reported. 
#Blobs with area less than this will be ignoder
area_crit = 100

#ID for the ball
ball_id = -1

#Global array that has a cache of tag IDs to blob color codes
arr_color_codes = []

#Global array where we are caching blobs
arr_blob_pos = []


#arr_tag_pos = []

class Tracker:

    def __init__(self):
	global arr_color_codes
	#Read the file with mapping of blobfinder color codes to tag IDs
	#As a result, we are treating blobs and tags same
        f = open('ColorCodesToIDs.txt', 'r')
        for line in f:
            if line.startswith("#"):
                continue
            codes = line.split()
            if len(codes) != 4:
                continue
	    #Transform all the fields to ints and make a tuple of them
	    t = (int(codes[0]), int(codes[1]), int(codes[2]), int(codes[3]))
            arr_color_codes.append(t);
	    
	    self.pub = rospy.Publisher(topic_name, Blobs)

    def track_blobs(self, blobs):
	global arr_blob_pos
	#Go over a complete list of blobs from the blobfinder
        for blob in arr_blob_pos:
	    blobs.blobs.append(blob)
	self.pub.publish(blobs)

    def localize(self, tags):
        global arr_blob_pos
	arr_blob_pos = []
	for tag in tags.tags:
	    for color_rec in arr_color_codes:
            	if (color_rec[3]== tag.id):
		    #print "Got a tag of interest"
		    #This blob should be converted to a tag. Do it
                    blob_from_tag = Blob()
		    blob_from_tag.red = color_rec[0]
		    blob_from_tag.green = color_rec[1]
		    blob_from_tag.blue = color_rec[2]
                    blob_from_tag.x = tag.x
                    blob_from_tag.y = tag.y
		    arrx = [tag.cwCorners[0], tag.cwCorners[2], tag.cwCorners[4], tag.cwCorners[6]]
		    arry = [tag.cwCorners[1], tag.cwCorners[3], tag.cwCorners[5], tag.cwCorners[7]]
		    minx = min(arrx)
		    maxx = max(arrx)
                    miny = min(arry)
                    maxy = max(arry)
		    if (minx < 0):
			minx = 0
		    if (maxx < 0):
			maxx = 0
		    if (miny < 0):
			miny = 0
		    if (maxy < 0):
			maxy = 0
		    blob_from_tag.left = int(minx)
		    blob_from_tag.top = int(miny)
		    blob_from_tag.right = int(maxx)
		    blob_from_tag.bottom = int(maxy)
	            arr_blob_pos.append(blob_from_tag)


if __name__ == '__main__':

    if len(sys.argv) == 2:
        topic_name = sys.argv[1];

    rospy.init_node('blob_ar_tracker')
    tracker = Tracker()

    rospy.Subscriber('tags', Tags, tracker.localize)
    rospy.Subscriber('blobs', Blobs, tracker.track_blobs)

    rospy.spin()

