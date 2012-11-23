#!/usr/bin/env python
import roslib; roslib.load_manifest('wiimote_twist')
import rospy
from joy.msg import Joy
from wiimote.msg import State
from geometry_msgs.msg import Twist



###############################################################################
### Globals

nunchuk = False
wiimoteState = False



###############################################################################
### Constants

MAXVEL_XLIN = 0.6 #m/s
MAXVEL_YLIN = 0.6 #m/s
MAXVEL_ZLIN = 1.5 #m/s
MAXVEL_ZANG = 1 #rad/s



###############################################################################
### Callback

def callbackNunchuk(n):
    global nunchuk
    nunchuk = n
    pass

def callbackWiimoteState(s):
    global wiimoteState
    wiimoteState = s
    pass



###############################################################################
### Main

def go():
    global nunchuk, wiimoteState
    global MAXVEL

    pub = rospy.Publisher('cmd_vel',Twist)
    rospy.init_node('node_name')
    rospy.Subscriber('wiimote/nunchuk', Joy, callbackNunchuk)
    rospy.Subscriber('wiimote/state', State, callbackWiimoteState)


    # wait for the nunchuk and wiimote to run
    r = rospy.Rate(30) # hz
    while not (nunchuk and wiimoteState): r.sleep()

    # main control loop
    while not rospy.is_shutdown():
        # publish
        t = Twist()
        t.linear.x = nunchuk.axes[1] * MAXVEL_XLIN
        t.linear.y = ( MAXVEL_YLIN if nunchuk.buttons[0] else # Z
                       -MAXVEL_YLIN if nunchuk.buttons[1] else # C
                       0)
        t.linear.z = ( MAXVEL_ZLIN if wiimoteState.buttons[4] else # A
                      -MAXVEL_ZLIN if wiimoteState.buttons[5] else # B
                      0)
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = nunchuk.axes[0] * MAXVEL_ZANG
        pub.publish(t)
        # wait for next loop
        r.sleep()

def dtrace(s,x):
    rospy.loginfo(s+" == %s"%x)
    return x;

if __name__ == '__main__':
    go()



