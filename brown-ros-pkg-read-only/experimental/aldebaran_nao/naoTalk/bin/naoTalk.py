#!/usr/bin/python
import sys
import roslib
roslib.load_manifest('naoTalk')
import rospy

from std_msgs.msg import String

def talker():
	speech = rospy.Publisher('speech', String)
	rospy.init_node('talker', anonymous=True)
	while not rospy.is_shutdown():
		str = sys.stdin.readline()
		speech.publish(String(str))
		rospy.sleep(1.0)

if __name__ == '__main__':
	print "Enter phrases to say:"
	try:
		talker()
	except:
		print 
