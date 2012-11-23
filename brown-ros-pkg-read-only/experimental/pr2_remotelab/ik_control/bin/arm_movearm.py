#! /usr/bin/env python/

# Python Arm Wrapper for the PR2
# Robert Bosch 

# Written by Sarah Osentoski and modeled off of code by Adam Stambler.

import roslib; roslib.load_manifest("ik_control")
import actionlib
from actionlib import simple_action_client
import actionlib_msgs.msg
import rospy
from message_conversion import poseConstraintToPositionOrientationConstraints
import move_arm_msgs.msg


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
    def __init__(self, armname, group_name, planner_servicename ):
        """ Example:    Left_arm = Arm("move_left_arm", "left_arm", "ompl_planning/plan_kinematic_path")"""
       
        self.arm = armname
        self.armClient = actionlib.SimpleActionClient(armname, move_arm_msgs.msg.MoveArmAction)

        self.armClient.wait_for_server()
       
        self.goalA=move_arm_msgs.msg.MoveArmGoal()
        self.goalA.motion_plan_request.group_name = group_name
        self.goalA.motion_plan_request.num_planning_attempts = 1
        self.goalA.motion_plan_request.planner_id = ""
        self.goalA.planner_service_name = planner_servicename
        self.goalA.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
        
            
    def getState(self):
        """Returns the current state of action client
        """
        return self.armClient.get_state()    


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

    def gotoAngle(self, angles):
        pass
#        """ Takes a list of angles sends a TrajectoryGoal action to the TrajectoryJointPlanner
#        """
#        if (len(angles) != len(self.joint_names)):
#            raise Exception("Wrong number of Angles. "+ len(angles) + "given.  "+ len(self.joint_names) + "needed.")
#    
#        trajMsg = pr2_controllers_msgs.msg.JointTrajectoryGoal()
#        
#        trajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
#        trajPoint.positions= angles
#        trajPoint.velocities = [0 for i in range(0, len(self.joint_names))]
#        trajPoint.time_from_start = rospy.Duration(1)
#            
#        trajMsg.trajectory.joint_names = self.joint_names
#        trajMsg.trajectory.points.extend([trajPoint])
#        self.sendTraj(trajMsg)


    def goToPoint(self, point, orientation=[0,0,0,1], tolerance=None, frame_id = None, link = None):
        """ Go to a cartisian point [x, y, z] in frame frame_id
            Link specifies the URDF link that must move to that coordinate
            With no link given, it defaults to the l _wrist_roll_link
        """
        if (frame_id == None):
            frame_id  = "torso_lift_link"
        if (link == None):
            link = 'l_wrist_roll_link'
        if (tolerance==None):
            tolerance=[.02, .02, .02, .04, .04, .04]
        pose = self.makePose(point, orientation, tolerance, frame_id, link)


        position_constraint, orientation_constraint= poseConstraintToPositionOrientationConstraints(pose);
        
        #may need a reset button...this might not be the answer
        self.goalA.motion_plan_request.goal_constraints.position_constraints=[]
        self.goalA.motion_plan_request.goal_constraints.position_constraints.append(position_constraint);
        self.goalA.motion_plan_request.goal_constraints.orientation_constraints=[]
        self.goalA.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint);



        finished_within_time=False
        self.armClient.send_goal(self.goalA)
        
        finished_within_time=self.armClient.wait_for_result(rospy.Duration(200))
        
        if not finished_within_time:
            self.armClient.cancel_goal()
            print "Timed out achieving goalA"
        else:
            state=self.armClient.get_state()
            success=state==actionlib_msgs.msg.GoalStatus.SUCCEEDED
            
            if success:
                
                print "Action Finished"
            else:
                print "Action Failed"
            
        return success      


    def makePose(self, xyz, orientation, tolerance, frame_id, link):
        """This is a shortcut method for making pose stamped messages
        """

        print xyz
        desired_pose=motion_planning_msgs.msg.SimplePoseConstraint()
        
        desired_pose.header.frame_id = frame_id
        desired_pose.link_name = link
        desired_pose.pose.position.x = xyz[0]
        desired_pose.pose.position.y = xyz[1]
        desired_pose.pose.position.z = xyz[2]
        
        desired_pose.pose.orientation.x = orientation[0]
        desired_pose.pose.orientation.y = orientation[1]
        desired_pose.pose.orientation.z = orientation[2]
        desired_pose.pose.orientation.w = orientation[3]
        
        desired_pose.absolute_position_tolerance.x = tolerance[0]
        desired_pose.absolute_position_tolerance.y = tolerance[1]
        desired_pose.absolute_position_tolerance.z = tolerance[2]

        desired_pose.absolute_roll_tolerance = tolerance[3]
        desired_pose.absolute_pitch_tolerance = tolerance[4]
        desired_pose.absolute_yaw_tolerance =  tolerance[5]

        return desired_pose

    
        
