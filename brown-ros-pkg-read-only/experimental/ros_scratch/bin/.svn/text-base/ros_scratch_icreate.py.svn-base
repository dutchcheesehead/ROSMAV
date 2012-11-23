#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_scratch')
import rospy
import socket
import re
from array import array
from irobot_create_2_1.msg import SensorPacket
from irobot_create_2_1.srv import Tank

HOST = '127.0.0.1'      # IP address of the remote host (in this case, the local machine)
                        # to connect to Scratch on a different computer, just put in its IP address
PORT = 42001            # The same port as used by the server

scratchSock=None



###############################################################################
# Input from Scratch
scratchBroadcastRE = re.compile((lambda afloat: 'broadcast\s\"left '+afloat+' right '+afloat+'\"')('(-?\d+(?:.\d*)?)'))
def parseData(str):
    global scratchBroadcastRE
    # Check for a broadcast
    # matches patterns like 'broadcast "left 34.5 right -64.2"'
    rospy.logdebug('Received str \'' + str + '\'')
    e = scratchBroadcastRE.search(str)
    if e:
        # We have a broadcast!
        # Nominal ranges for tank: -100 to 100 in Scratch.  The driver irobot_create_2_1
        #   measures velocity in mm/s, with a maximum of 500 mm/s.
        left=int(float(e.group(1))*5)
        right=int(float(e.group(2))*5)
        tank(left,right)

def tank(left, right):
    # Call the tank service for l,r
    rospy.wait_for_service('tank')
    try:
        srv = rospy.ServiceProxy('tank', Tank)
        result = srv(True,left,right) #clear,left,right
    except rospy.ServiceException, e:
        rospy.logerr("ROS Service call (to irobot_create_2_1 Tank service) failed: %s"%e)



###############################################################################
# Output to Scratch
def sendScratchSensor(variable, value, scratchSock):
    sendScratchCommand('sensor-update \"'+variable+'\"'+' '+value+' ', scratchSock)

def sendScratchCommand(cmd, scratchSock):
    n = len(cmd)
    a = array('c')
    a.append(chr((n >> 24) & 0xFF))
    a.append(chr((n >> 16) & 0xFF))
    a.append(chr((n >>  8) & 0xFF))
    a.append(chr(n & 0xFF))
    scratchSock.send(a.tostring() + cmd)



###############################################################################
# Callbacks
def cb_sensorPacket(sp):
    global scratchSock
    if not scratchSock:
        return
    sendScratchSensor("rosScratchBump",
                      "1" if (sp.bumpLeft or sp.bumpRight) else "0",
                      scratchSock)
    sendScratchSensor("rosScratchLightLeft",  str(sp.cliffFrontLeftSignal),  scratchSock)
    sendScratchSensor("rosScratchLightRight", str(sp.cliffFrontRightSignal), scratchSock)



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
    rospy.init_node('ros_scratch_icreate')
    rospy.Subscriber('sensorPacket', SensorPacket, lambda sp: cb_sensorPacket(sp))
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
            parseData(data)

    if scratchSock:
        scratchSock.close()

if __name__ == '__main__':
    main()



