#!/usr/bin/python

import roslib; roslib.load_manifest('faces')
import rospy
import sys
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from NAOcontrol.msg import Head

class Face_detector:

    def __init__(self):
        # Width of window to apply face search.  Height will be scaled
        # appropriately, based on input image.
        self.x_search_size = 320.0
        self.cascade = None
        cascade_file = rospy.get_param("/faces/cascade", "/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")
        self.cascade = cv.Load(cascade_file)
        if not self.cascade:
            print("Error: Could not load classifier cascade")
            sys.exit(-1)
        self.storage = cv.CreateMemStorage(0)
        self.haar_scale = 1.2
        self.min_neighbors = 2
        self.haar_flags = cv.CV_HAAR_DO_CANNY_PRUNING
        self.min_size = (20,20)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/faces/image", Image)
        self.motion_pub = rospy.Publisher("head", Head)
        self.lock = False

    def detect(self, image):
        if self.lock:
            return
        self.lock = True
        try:
            img = self.bridge.imgmsg_to_cv(image, "bgr8")
        except cvBridgeError, e:
            print e
        image_scale = img.width / self.x_search_size
        small_img = cv.CreateImage((cv.Round(img.width/image_scale), cv.Round(img.height/image_scale)), 8, 1)
        gray = cv.CreateImage((img.width, img.height), 8, 1)
        cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
        cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)
        cv.EqualizeHist(small_img, small_img)
        faces = cv.HaarDetectObjects(small_img, self.cascade, self.storage, self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
        if faces:
            head = Head()
            for ((x, y, width, height), neighbors) in faces:
                upper_left = (int(x*image_scale), int(y*image_scale))
                lower_right = (int((x+width)*image_scale), int((y+height)*image_scale))
                cv.Rectangle(img, upper_left, lower_right, cv.RGB(255,0,0), 3, 8, 0)
                head.x = x
                head.y = y
            self.motion_pub.publish(head)
        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(img, "bgr8"))
        except CvBridgeError, e:
            print e
        self.lock = False

if __name__ == "__main__":
    rospy.init_node("faces")
    fd = Face_detector()
    image_topic = rospy.get_param("/faces/image_topic", "/gscam/image_raw")
    rospy.Subscriber(image_topic, Image, fd.detect)
    rospy.spin()

