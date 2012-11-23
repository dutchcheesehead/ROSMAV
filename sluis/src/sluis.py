#!/usr/bin/env python
import roslib
roslib.load_manifest('sluis')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

def sluis(van, naar, t):
	rospy.Subscriber(van, t, rospy.Publisher(naar, t).publish)

rospy.init_node('sluis', anonymous=True)

sluis("/ardrone/image_raw", "/usb_cam/image_raw", Image)
sluis("/ardrone/camera_info", "/usb_cam/camera_info", CameraInfo)

try:
	rospy.spin()
except KeyboardInterrupt:
	print "Shutting down"
