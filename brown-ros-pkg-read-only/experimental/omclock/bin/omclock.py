#!/usr/bin/env python
import roslib; roslib.load_manifest('omclock')
import rospy, time
from math import cos,sin,pi
from om_msgs.msg import Overhead_Map_Obj, Overhead_Map_Objs

def go():
    pub = rospy.Publisher('overhead_map_objs',Overhead_Map_Objs)
    rospy.init_node('omclock')

    # main control loop
    r = rospy.Rate(10) # hz
    while not rospy.is_shutdown():
        # Get time.  Make an analog clock face with center (centerX,centerY)
        #   and largest radius "radius"
        # This one's set up for the soccer field's center
        centerX = 2
        centerY = 1.25
        radius = 1
        curTime = time.localtime();

        hours = curTime.tm_hour
        minutes = curTime.tm_min
        seconds = curTime.tm_sec + (time.time()%1) # for partial seconds

        # Hour hand is an arrow
        hourAngle = clockAngle((hours+minutes*1./60+seconds*1./3600)*1./12)
        hourHand = Overhead_Map_Obj()
        hourHand.name = "arrow"
        hourHand.tuple = [centerX,
                          centerY,
                          centerX+0.5*radius*cos(hourAngle),
                          centerY+0.5*radius*sin(hourAngle)]
        # Minute hand is a line
        minuteAngle = clockAngle((minutes+seconds*1./60)*1./60)
        minuteHand = Overhead_Map_Obj()
        minuteHand.name = "line"
        minuteHand.tuple = [centerX,
                            centerY,
                            centerX+0.75*radius*cos(minuteAngle),
                            centerY+0.75*radius*sin(minuteAngle)]
        # Second "hand" is a robot going around the outside
        secondAngle = clockAngle(seconds*1./60)
        secondHand = Overhead_Map_Obj()
        secondHand.name = "irobot-create"
        secondHand.tuple = [centerX+radius*cos(secondAngle),
                            centerY+radius*sin(secondAngle),
                            secondAngle-pi/2]
        # Center "dot"
        centerDot = Overhead_Map_Obj()
        centerDot.name = "point"
        centerDot.tuple = [centerX, centerY]

        # Put it all together (front drawn first)
        drawing = Overhead_Map_Objs()
        drawing.objs = [secondHand, minuteHand, hourHand, centerDot]
        pub.publish(drawing)
        # wait for next loop
        r.sleep()

# Input: propTime in [0,1]
# propTime is, eg, hours/12 or minutes/60
# Returns the angle (in radians) of the clock pointer at
def clockAngle(propTime):
    return (2*pi*(1-propTime)) + pi/2

def dtrace(s,x):
    rospy.loginfo(s+" == %s"%x)
    return x;

if __name__ == '__main__':
    go()



