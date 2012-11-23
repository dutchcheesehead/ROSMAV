#!/usr/bin/env python
import roslib
roslib.load_manifest('arwen')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#get these guys in the manifest eventually
import sys
import ctypes
import numpy
import array
import time

if __name__ == '__main__':
	libc = ctypes.cdll.LoadLibrary("libmesasr.so")

	sr = ctypes.c_int()

	print "\n\n";

	if (libc.SR_OpenUSB(ctypes.byref(sr), 0) <= 0): #zero for autodiscover
		print "Couldn't reach the Swiss Ranger."
		sys.exit()
	
	sn = libc.SR_ReadSerial(sr)
	print "Connected to Swiss Ranger no. %i 0x%08x" % (sn,sn)

	height = libc.SR_GetRows(sr)
	width = libc.SR_GetCols(sr)

	print "resolution: %s x %s" % (width,height)

	img = cv.CreateImage((width,height), cv.IPL_DEPTH_8U, 1);
	dis_img = cv.CreateImage((width,height), cv.IPL_DEPTH_8U, 1);
	sr_img = array.array('H', [0] * width*height)


	rospy.init_node('arwen', anonymous=True)

	pub = rospy.Publisher("depth/image",Image)

	bridge = CvBridge()

	try:

		while True:
			libc.SR_Acquire(sr)

			ctypes.memmove(sr_img.buffer_info()[0], libc.SR_GetImage(sr,0), width*height*2);

			for x in xrange(width):
				for y in xrange(height):
					z = sr_img[x+y*width]
					z = int(z / 258)
					img[y,x] = z

			cv.Resize(img,dis_img)
			pub.publish(bridge.cv_to_imgmsg(dis_img, "mono8"))

	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		pass
