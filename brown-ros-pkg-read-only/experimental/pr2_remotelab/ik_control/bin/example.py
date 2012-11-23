import roslib; roslib.load_manifest("ik_control")
import actionlib
import actionlib.msg
import actionlib_msgs.msg
import rospy
import move_arm_msgs.msg
import motion_planning_msgs.msg
import geometric_shapes_msgs.msg

def poseConstraintToPositionOrientationConstraints(pose_constraint):
    position_constraint=motion_planning_msgs.msg.PositionConstraint()
    position_constraint.header=pose_constraint.header
    position_constraint.link_name = pose_constraint.link_name
    position_constraint.position = pose_constraint.pose.position
    position_constraint.constraint_region_shape.type = geometric_shapes_msgs.msg.Shape.BOX
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

    position_constraint.constraint_region_orientation.x = 0.0
    position_constraint.constraint_region_orientation.y = 0.0
    position_constraint.constraint_region_orientation.z = 0.0
    position_constraint.constraint_region_orientation.w = 1.0

    position_constraint.weight = 1.0

    orientation_constraint=motion_planning_msgs.msg.OrientationConstraint()
    orientation_constraint.header = pose_constraint.header
    orientation_constraint.link_name = pose_constraint.link_name
    orientation_constraint.orientation = pose_constraint.pose.orientation
    orientation_constraint.type = pose_constraint.orientation_constraint_type

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
    orientation_constraint.weight = 1.0

    return position_constraint, orientation_constraint






if __name__=="__main__":

    rospy.init_node('test_example', anonymous=True)
    move_arm=actionlib.SimpleActionClient("move_left_arm", move_arm_msgs.msg.MoveArmAction)
    move_arm.wait_for_server()
    print "Connected to server"
    goalA=move_arm_msgs.msg.MoveArmGoal()
    
    goalA.motion_plan_request.group_name = "left_arm"
    goalA.motion_plan_request.num_planning_attempts = 1
    goalA.motion_plan_request.planner_id = ""
    goalA.planner_service_name = "ompl_planning/plan_kinematic_path"
    goalA.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)

    desired_pose=motion_planning_msgs.msg.SimplePoseConstraint()

    desired_pose.header.frame_id = "torso_lift_link"
    desired_pose.link_name = "l_wrist_roll_link"
    desired_pose.pose.position.x = .6
    desired_pose.pose.position.y = .4
    desired_pose.pose.position.z = .2

    desired_pose.pose.orientation.x = 0.0
    desired_pose.pose.orientation.y = 0.0
    desired_pose.pose.orientation.z = 0.0
    desired_pose.pose.orientation.w = 1.0

    desired_pose.absolute_position_tolerance.x = 0.02
    desired_pose.absolute_position_tolerance.y = 0.02
    desired_pose.absolute_position_tolerance.z = 0.02

    desired_pose.absolute_roll_tolerance = 0.04
    desired_pose.absolute_pitch_tolerance = 0.04
    desired_pose.absolute_yaw_tolerance = 0.04

    #don't think I can add a .h file to python so replicating that code here
    # addGoalConstraintToMoveArmGoal
    
    position_constraint, orientation_constraint= poseConstraintToPositionOrientationConstraints(desired_pose);
    goalA.motion_plan_request.goal_constraints.position_constraints.append(position_constraint);
    goalA.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint);
    # ----------end addGoalConstraintToMoveArmGoal

    
    while not rospy.is_shutdown():
        finished_within_time=False
        move_arm.send_goal(goalA)
        finished_within_time=move_arm.wait_for_result(rospy.Duration(200))
        if not finished_within_time:
            move_arm.cancel_goal()
            print "Timed out achieving goalA"
        else:
            state=move_arm.get_state()
            success=state==actionlib_msgs.msg.GoalStatus.SUCCEEDED
            if success:
                print "Action Finished"
            else:
                print "Action Failed"
                
                                             
    
