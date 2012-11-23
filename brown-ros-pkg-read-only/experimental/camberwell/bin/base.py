#!/usr/bin/python
import roslib; roslib.load_manifest('camberwell')
import rospy
from std_msgs.msg import String
from random import shuffle

pub = None;
deck = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")

def processMsg(data):
	shuffle(deck)
	payload = deck[0]*bytesPerMsg
	pub.publish(String(payload))

if __name__ == "__main__":
	rospy.init_node('camberwell')
	topic = rospy.get_param('/camberwell/topic','/camberwell')
	bytesPerMsg = rospy.get_param('/camberwell/bytesPerMsg',131072)

	pub = rospy.Publisher(topic,String)

	print "Waiting for publisher..."
	rospy.sleep(5.)
	pub.publish(String(topic))

	print "Waiting for initial topic..."
	rospy.sleep(5.)
	rospy.Subscriber(topic, String, processMsg)

	print "Waiting for subscription..."
	rospy.sleep(5.)
	pub.publish(String("echo."))

	print "ready to measure"
	rospy.spin()
