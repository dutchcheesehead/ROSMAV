#!/usr/bin/env python
import roslib; roslib.load_manifest('line_follower')
import rospy
from irobot_create_2_1.msg import SensorPacket
from geometry_msgs.msg import Twist

# Assumes line is "bright" and rest of floor is "dark"

# globals
newInfo = False
# updated on receiving data
lSensorVal = 0
lfSensorVal = 0
rfSensorVal = 0
rSensorVal = 0
bump = False


def receiveSensorUpdate(sensorPacket):
    global newInfo
    global lSensorVal
    global lfSensorVal
    global rfSensorVal
    global rSensorVal
    global bump
    lSensorVal = sensorPacket.cliffLeftSignal
    lfSensorVal = sensorPacket.cliffFrontLeftSignal
    rfSensorVal = sensorPacket.cliffFrontRightSignal
    rSensorVal = sensorPacket.cliffRightSignal
    bump = sensorPacket.bumpLeft or sensorPacket.bumpRight
    newInfo = True

def steerRobot(pub,speed,rotation):
    # twist.linear.x is speed; positive is forward
    # twist.angular.z is rotation; positive is CCW
    twist = Twist()
    twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = rotation
    pub.publish(twist)

def go():
    global newInfo
    global lSensorVal
    global lfSensorVal
    global rfSensorVal
    global rSensorVal
    global bump

    # constants
    maxSpeed = 0.075
    speedDecayRate = 1.5 # makes speed -> speed/speedDecayRate
    maxTurn = 0.5

    # init
    pub = rospy.Publisher('cmd_vel',Twist)
    rospy.init_node('line_follower')
    rospy.Subscriber('sensorPacket', SensorPacket, receiveSensorUpdate)

    # calibration
    rospy.loginfo("Calibrating sensors now.  Make sure robot is over line.")
    lMin = 1e100; lMax = 0
    lfMin = 1e100; lfMax = 0
    rfMin = 1e100; rfMax = 0
    rMin = 1e100; rMax = 0
    steerRobot(pub,0,maxTurn)
    for i in range(100): # magic constant; hopefully this will be enough to pick up numbers.
        while not newInfo: pass
        newInfo = False
        lMin = min(lMin,lSensorVal) ; lMax = max(lMax,lSensorVal)
        lfMin = min(lfMin,lfSensorVal) ; lfMax = max(lfMax,lfSensorVal)
        rfMin = min(rfMin,rfSensorVal) ; rfMax = max(rfMax,rfSensorVal)
        rMin = min(rMin,rSensorVal) ; rMax = max(rMax,rSensorVal)
    steerRobot(pub,0,0) # stop!
    raw_input("Calibration complete.  Steer robot so that front\n  left sensor is on line and front right is not, and press enter to continue.")

    # get line type; set thresholds.
    #   - thresholds are overlapping, so we don't have the "back and forth" problem
    while not newInfo: pass
    rospy.loginfo("Assuming robot's front left sensor is on the line and its right sensor is off the line... go!")
    lineIsBright = (lfSensorVal > rfSensorVal)
    rospy.loginfo("sensors read: left was %s, right was %s"%(lfSensorVal,rfSensorVal))
    lThresholdBrightToDark = dtrace("lthreshB->D", lMin + 0.4*(lMax-lMin))
    lThresholdDarkToBright = dtrace("lthreshD->B", lMin + 0.6*(lMax-lMin))
    lfThresholdBrightToDark = dtrace("lfthreshB->D", lfMin + 0.4*(lfMax-lfMin))
    lfThresholdDarkToBright = dtrace("lfthreshD->B", lfMin + 0.6*(lfMax-lfMin))
    rfThresholdBrightToDark = dtrace("rfthreshB->D", rfMin + 0.4*(rfMax-rfMin))
    rfThresholdDarkToBright = dtrace("rfthreshD->B", rfMin + 0.6*(rfMax-rfMin))
    rThresholdBrightToDark = dtrace("rthreshB->D", rMin + 0.4*(rMax-rMin))
    rThresholdDarkToBright = dtrace("rthreshD->B", rMin + 0.6*(rMax-rMin))

    # State variables
    wasBumped = False # during last iteration, at least one bump sensor was on
    unBumpedSpeed = 0; unBumpedTurn = 0
    speed = 0 # no speed to start
    turn = maxTurn # go CCW to start
    # (one-sensor only)
    wasOnLine = True # during last iteration, robot was on the line
    # (two-sensor only)
    wasOnLineLeft = True # if we were hitting the line with the left sensor
    wasOnLineRight = False # if we were hitting the line with the right sensor

    # Go!
    steerRobot(pub,speed,turn)
    # loop between getting data and pushing out a new action.  The robot will:
    #  - twist rightwards ("twistRight") if on the line, in an effort to get off of it
    #  - push the forward, turning leftward, ("pushLeft") if off the line,
    #      in an effort to get back on it
    while not rospy.is_shutdown():
        while not newInfo: pass
        newInfo = False
        if bump:
            if not wasBumped:
                # Boston driver response
                rospy.loginfo("Hey, watch where you're going!")
                unBumpedSpeed = dtrace("speed",speed)
                unBumpedTurn = dtrace("turn",turn)
            wasBumped = True
            # halt!
            speed = 0
            turn = 0
        elif wasBumped:
            rospy.loginfo("Outta my way!")
            wasBumped = False
            speed = unBumpedSpeed
            turn = unBumpedTurn
        else: # not bump
            if False: # one-sensor following: front left sensor (CONSTANT)
                if wasOnLine:
                    if ((lineIsBright and (lfSensorVal < lfThresholdBrightToDark)) or
                        ((not lineIsBright) and (lfSensorVal > lfThresholdDarkToBright))):
                        rospy.loginfo("Left the line!")
                        wasOnLine = False
                        speed = maxSpeed
                        turn = -maxTurn
                    else: # haven't left the line yet; slow down more
                        speed = speed/speedDecayRate
                else: # not wasOnLine
                    if ((lineIsBright and (lfSensorVal > lfThresholdDarkToBright)) or
                        ((not lineIsBright) and (lfSensorVal < lfThresholdBrightToDark))):
                        rospy.loginfo("Got back on the line!")
                        wasOnLine = True
                        speed = maxSpeed
                        turn = maxTurn
                    else: # haven't gotten back on the line yet; slow down more
                        speed /= speedDecayRate
            else: # two-sensor following: far left and right sensors
                if wasOnLineLeft:
                    if ((lineIsBright and (lfSensorVal < lfThresholdBrightToDark)) or
                        ((not lineIsBright) and (lfSensorVal > lfThresholdDarkToBright))):
                        rospy.loginfo("Left the line (left sensor)!")
                        wasOnLineLeft = False
                        speed = maxSpeed
                        turn = 0
                    else: # haven't left the line yet; slow down more
                        speed = speed/speedDecayRate
                elif wasOnLineRight:
                    if ((lineIsBright and (rfSensorVal < rfThresholdBrightToDark)) or
                        ((not lineIsBright) and (rfSensorVal > rfThresholdDarkToBright))):
                        rospy.loginfo("Left the line! (right sensor)")
                        wasOnLineRight = False
                        speed = maxSpeed
                        turn = 0
                    else: # haven't left the line yet; slow down more
                        speed = speed/speedDecayRate
                else: # not wasOnLine{Left,Right}
                    if ((lineIsBright and (lfSensorVal > lfThresholdDarkToBright)) or
                        ((not lineIsBright) and (lfSensorVal < lfThresholdBrightToDark))):
                        rospy.loginfo("Got back on the line (left sensor)!")
                        wasOnLineLeft = True
                        speed = 0
                        turn = maxTurn
                    elif ((lineIsBright and (rfSensorVal > rfThresholdDarkToBright)) or
                          ((not lineIsBright) and (rfSensorVal < rfThresholdBrightToDark))):
                        rospy.loginfo("Got back on the line (right sensor)!")
                        wasOnLineRight = True
                        speed = 0
                        turn = -maxTurn
                    else: # haven't gotten back on the line yet; keep going!
                        speed = maxSpeed
                        turn = 0
        steerRobot(pub,speed,turn)

            # Stop!
    steerRobot(pub,0,0)

def dtrace(s,x):
    rospy.loginfo(s+" == %s"%x)
    return x;

if __name__ == '__main__':
    go()



