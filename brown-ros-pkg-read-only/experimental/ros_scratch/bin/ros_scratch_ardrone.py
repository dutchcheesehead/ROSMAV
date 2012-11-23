#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_scratch')
import rospy
import socket
import re
from array import array
from geometry_msgs.msg import Twist
from ar_recog.msg import Tag, Tags

HOST = '127.0.0.1'      # IP address of the remote host (in this case, the local machine)
                        # to connect to Scratch on a different computer, just put in its IP address
PORT = 42001            # The same port as used by the server

scratchSock=None

###############################################################################
# Input from Scratch
scratchBroadcastRE = re.compile((lambda afloat: 'broadcast\s\"left '+afloat+' right '+afloat+'\"')('(-?\d+(?:.\d*)?)'))
def parseData(str, pub):
    # Check for a broadcast
    # matches patterns like 'broadcast "left 34.5 right -64.2"'
    rospy.logdebug('Received str \'' + str + '\'\n')
    e = scratchBroadcastRE.search(str)
    if e:
        # We have a broadcast!
        # Nominal ranges for tank: -100 to 100 in Scratch.
        left=int(float(e.group(1)))
        right=int(float(e.group(2)))
        ardrone_tank(pub, left, right)

def ardrone_tank(pub, left, right):
    # Fake a "tank" service for the AR.Drone
    # Assume that left, right range from -100 to 100, and
    #   that the approximate maximum velocity of the ardrone
    #   is 0.5 m/s linear and 4 rad/s angular.
    # Enforce input range
    left = max(min(left,100),-100)
    right = max(min(right,100),-100)
    # The method for finding velocities here may not be principled (I honestly
    #   haven't checked.) but should suffice for demonstration.
    t = Twist()
    t.linear.x = (left+right) * 0.0025 # 0.0025: Shrink range from [-200,200] to [-0.5,0.5]
    t.linear.y = 0
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = (right-left) * 0.02 # 0.02: Shrink range from [-200,200] to [-4,4]
    pub.publish(t)



###############################################################################
# Make a connection to Scratch
def makeConnection():
    global scratchSock
    try:
        rospy.loginfo("Connecting to Scratch...")
        scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        scratchSock.connect((HOST, PORT))
        rospy.loginfo("Connected to Scratch!")
    except socket.error, e:
        scratchSock = None
        rospy.logwarn("Could not open socket to connect to Scratch: %s"%e)
        rospy.sleep(1.0)


###############################################################################
# Main
def main():
    global scratchSock

    # Connect to ROS
    rospy.loginfo("Connecting to ROS...")
    rospy.init_node('ros_scratch_ardrone')
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.loginfo("Connected to ROS!")

    # Receive Scratch commands;
    while not rospy.is_shutdown():
        while(scratchSock is None):
            makeConnection()
        try:
            data = scratchSock.recv(1024)
        except socket.error, e:
            scratchSock = None
            rospy.logwarn("Could not receive data from Scratch socket: %s"%e)
        if not data:
            scratchSock=None
            rospy.logwarn("Lost connection to Scratch.")
        else:
            parseData(data, pub)

    if scratchSock:
        scratchSock.close()

if __name__ == '__main__':
    main()



