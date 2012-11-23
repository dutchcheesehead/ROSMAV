#! /usr/bin/env python/

# Python Arm Wrapper for the PR2
# Robert Bosch 

# Written by Adam Stambler with tons of help from the ros docs
# Modifed by Sarah Osentoski to use IK w/ collision checking, to take in rotation information

import roslib; roslib.load_manifest("ik_control")
import actionlib
import rospy
import pr2_controllers_msgs.msg
import geometry_msgs.msg
import kinematics_msgs.msg
import kinematics_msgs.srv
import motion_planning_msgs.msg
import motion_planning_msgs.srv
import trajectory_msgs.msg 
import std_msgs.msg
from math import *
import numpy as np
import tf


class Arm:
    def __init__(self, controllerPath, ikPath ):
        """ Initialize a wrapper around a PR2 arm.  This class allows easy joint control of a robot arm.
            controllerPath -   namespace of the jointTrajectory controller
            ikpath        -  namespace of the inverse kinematics controller
            
            Example:
            Right_arm = Arm("/r_arm_controller", "/pr2_right_arm_kinematics")
            
            This creates a action client on :
                /r_arm_controller//joint_trajectory_action
            A service proxy at :
                /pr2_right_arm_kinematics/get_ik
        """
       
        self.controller = controllerPath
        self.trajClient = actionlib.SimpleActionClient(self.controller+"/joint_trajectory_action", pr2_controllers_msgs.msg.JointTrajectoryAction)
        
        self.joint_names, self.cAngles = self.getJointState()
        
        self.ikPath = ikPath
        #self._runIK = rospy.ServiceProxy(ikPath + "/get_constraint_aware_ik", kinematics_msgs.srv.GetConstraintAwarePositionIK)
        self._runIK = rospy.ServiceProxy(ikPath + "/get_ik", kinematics_msgs.srv.GetPositionIK)
       
        # self._getIKInfo = rospy.ServiceProxy(ikPath+ "/get_ik_solver_info", kinematics_msgs.msg.KinematicSolverInfo)
        
    def sendTraj(self, traj):
        """ Send the joint trajectory goal to the action server"""
        self.trajClient.send_goal(traj)
    
    def getState(self):
        """Returns the current state of action client
        """
        return self.trajClient.get_state()    
    def getJointNames(self):
        """ Retrieve a list of the joint names being controlled by this arm
        """
        return self.joint_names
    
    def getJointState(self):
        """ Contacts joint state action server and grabs joint state message"""
        msg=  rospy.wait_for_message(self.controller+ "/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState)
        return msg.joint_names, msg.actual.positions
  

    def getJointAngles(self):
        """ Returns a list of current joint angles.  Joint angles are listed in the same order
        as the joint_name list.
        """
        tmp, angles = self.getJointState()
        return angles
    def getIK(self, goalPose, link_name , seedAngles):
        """Calls inverse kinematics service for the arm. 
            goalPose -  Pose Stamped message showing desired position of link in coord system
            linkName - goal Pose target link name -  (r_wrist_roll_link or l_wrist_roll_link)
            startAngles -  seed angles for IK
        """
        ikreq = kinematics_msgs.msg.PositionIKRequest()
        ikreq.ik_link_name =link_name
        ikreq.pose_stamped = goalPose
        ikreq.ik_seed_state.joint_state.name = self.joint_names
        ikreq.ik_seed_state.joint_state.position = seedAngles
        ikres  = self._runIK(ikreq, rospy.Duration(5))
        

        #req=kinematics_msgs.srv.GetConstraintAwarePositionIK._request_class()
        #print dir(req)
        
        #ikreq = kinematics_msgs.msg.PositionIKRequest()
        #req.ik_request.ik_link_name =link_name
        #req.ik_request.pose_stamped = goalPose
        #req.ik_request.ik_seed_state.joint_state.name = self.joint_names
        #req.ik_request.ik_seed_state.joint_state.position = seedAngles
        #req.timeout=rospy.Duration(5)
        #ikres  = self._runIK(req)#, rospy.Duration(5))

        if (ikres.error_code.val != ikres.error_code.SUCCESS): #figure out which error code
            raise rospy.exceptions.ROSException("Could not find IK with " + self.ikPath + "\n\n" + ikres.__str__())


        return ikres
#return ikres
    def gotoAngle(self, angles):
        """ Takes a list of angles sends a TrajectoryGoal action to the TrajectoryJointPlanner
        """
        if (len(angles) != len(self.joint_names)):
            raise Exception("Wrong number of Angles. "+ len(angles) + "given.  "+ len(self.joint_names) + "needed.")
    
        trajMsg = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        
        trajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
        trajPoint.positions= angles
        trajPoint.velocities = [0 for i in range(0, len(self.joint_names))]
        trajPoint.time_from_start = rospy.Duration(1)
            
        trajMsg.trajectory.joint_names = self.joint_names
        trajMsg.trajectory.points.extend([trajPoint])
        self.sendTraj(trajMsg)

    def goToPoint(self, point, orientation=[0,0,0,1], frame_id = None, link = None):
        """ Go to a cartisian point [x, y, z] in frame frame_id
            Link specifies the URDF link that must move to that coordinate
            With no link given, it defaults to the l or r _wrist_roll_link
        """
        if (frame_id == None):
            frame_id  = "torso_lift_link"
        if (link == None):
            link = self.controller[1] + '_wrist_roll_link'          
        pose = self.makePose(point, orientation, frame_id)
        try:
            ik  = self.getIK(pose, link, self.getJointAngles())
            self.gotoAngle(ik.solution.joint_state.position)
        except:
            print "Could not move to that positon"

    def makePose(self, xyz, orientation, frameID):
        """This is a shortcut method for making pose stamped messages
        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz[0], xyz[1], xyz[2]
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1] 
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        pose.header.frame_id = frameID
        pose.header.stamp = rospy.Time.now()
        return pose
