#!/usr/bin/env python

import roslib
import signal
roslib.load_manifest('nao2pr2')

import rospy
import os

from NAOcontrol.msg import *
from NAOarm.msg import *
from trajectory_msgs.msg import *
from pr2_mechanism_msgs.msg import *
from pr2_mechanism_msgs.srv import *
from pr2_controllers_msgs.msg import *

import sys
import time

#initialize the arm and gripper controllers
pub_right = rospy.Publisher('r_arm_controller/command', JointTrajectory, latch=True)
pub_left = rospy.Publisher('l_arm_controller/command', JointTrajectory, latch=True)
rhand_pub = rospy.Publisher('r_gripper_controller/command', Pr2GripperCommand)
lhand_pub = rospy.Publisher('l_gripper_controller/command', Pr2GripperCommand)
head_pub = rospy.Publisher('head_traj_controller/command', JointTrajectory, latch=True)

#initialize the open gripper command
open_cmd = Pr2GripperCommand()
open_cmd.position = 0.08     #open position
open_cmd.max_effort = -1.0   #no limit on force

#initialize the close gripper command
close_cmd = Pr2GripperCommand()
close_cmd.position = -100.00   #closed position
close_cmd.max_effort = -1.0    #no limit on force

rhand_state = True
lhand_state = True


usage = """
No paramters.
"""
      

def go(side, positions):
  traj = JointTrajectory()
  traj.joint_names = ["%s_shoulder_pan_joint" % side,
                      "%s_shoulder_lift_joint" % side,
                      "%s_upper_arm_roll_joint" % side,
                      "%s_elbow_flex_joint" % side,
                      "%s_forearm_roll_joint" % side,
                      "%s_wrist_flex_joint" % side,
                      "%s_wrist_roll_joint" % side]
  traj.points = []
  for p in positions:
    traj.points.append(JointTrajectoryPoint(positions = p[1:],
                                            velocities = [0.0] * (len(p) - 1),
                                            accelerations = [],
                                            time_from_start = rospy.Duration(p[0])))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.001)
  if(side == 'l'):
    pub_left.publish(traj)
  else:
    pub_right.publish(traj)

  
def handleLeftArm(msg):
  global lhand_state
  positions = [[0.0,msg.shoulder_roll,msg.shoulder_pitch,msg.elbow_yaw + 1.57,msg.elbow_roll,msg.wrist_yaw,0.0,0.0]]
  go('l', positions)
  if(lhand_state != msg.hand):
    lhand_state = msg.hand
    if(not lhand_state): #should I open or close the hand
      #here I open the hand
      lhand_pub.publish(open_cmd)
    else:
      #here I close the hand
      lhand_pub.publish(close_cmd)
    
def handleRightArm(msg):
  global rhand_state
  positions = [[0.0,msg.shoulder_roll,msg.shoulder_pitch,msg.elbow_yaw - 1.57,-msg.elbow_roll,msg.wrist_yaw,0.0,0.0]]
  go('r', positions)
  if(rhand_state != msg.hand):
    rhand_state = msg.hand
    if(not rhand_state): #should I open or close the hand
      #here I open the hand
      rhand_pub.publish(open_cmd)
    else:
      #here I close the hand
      rhand_pub.publish(close_cmd)

def handleHead(msg):
  p = [msg.x,msg.y]
  traj = JointTrajectory()
  traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
  traj.points = []
  traj.points.append(JointTrajectoryPoint(positions = p,
                                            velocities = [0.0] * (len(p)),
                                            accelerations = [],
                                            time_from_start = rospy.Duration(0.0)))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.001)
  head_pub.publish(traj)


if __name__ == '__main__':


  rospy.init_node('nao2pr2', anonymous = False)
  rospy.wait_for_service('pr2_controller_manager/switch_controller')
  
  print "Subscribing to nao messages\n"
  rospy.Subscriber("cmd_RArm", Arm, handleRightArm)
  rospy.Subscriber("cmd_LArm", Arm, handleLeftArm)
  rospy.Subscriber("headF",HeadF, handleHead)
  rospy.spin()






