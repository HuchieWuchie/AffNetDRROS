#!/usr/bin/env python
import sys
import copy
import rospy
import argparse
from moveit_scripts.srv import *
from moveit_scripts.msg import *
import moveit_commander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import *
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
# from geometry_msgs.msg import Pose
import std_msgs.msg
from std_msgs.msg import Int8
from math import pi
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf2_ros
# from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


def get_ik(pose_msg, robot_state):
    pose_msg_stamped = geometry_msgs.msg.PoseStamped()
    pose_msg_stamped.header.frame_id = "world"
    pose_msg_stamped.header.stamp = rospy.Time.now()
    pose_msg_stamped.pose = pose_msg

    request_msg = moveit_msgs.msg.PositionIKRequest()
    request_msg.group_name = "manipulator"
    request_msg.robot_state = robot_state
    request_msg.avoid_collisions = True #False
    request_msg.pose_stamped = pose_msg_stamped
    request_msg.timeout = rospy.Duration(1.0) #
    request_msg.attempts = 10
    try:
        robot_state_out = calculate_ik(request_msg)
        return robot_state_out
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def validate_poses(poses_list):
    isFeasable = False
    robot_states = []
    #state_at_start = ready_state
    state_at_waypoint = get_ik(poses_list[0], ready_state)
    if state_at_waypoint.error_code.val == 1:
        state_at_goal = get_ik(poses_list[1], state_at_waypoint.solution)
        if state_at_goal.error_code.val == 1:
            isFeasable = True
            robot_states = [state_at_waypoint, state_at_goal]

    return isFeasable, robot_states


def transform_frame(req):
    print("Transforming message to the world frame")
    transformed_path_msg = nav_msgs.msg.Path()
    transformed_path_msg.header.frame_id = "world"
    tf_buffer.lookup_transform(req.header.frame_id, 'world', rospy.Time.now(), rospy.Duration(1.0))
    for i in range(len(req.poses)):
        transformed_pose_msg = geometry_msgs.msg.PoseStamped()
        transformed_pose_msg = tf_buffer.transform(req.poses[i], "world")
        transformed_path_msg.poses.append(transformed_pose_msg)
        transformed_path_msg.poses[-1].header.frame_id = req.poses[i].header.frame_id
    transformed_path_msg.header.stamp = rospy.Time.now()
    return trasnformed_path_msg


def move_to_goal(path_msg):
    global ready_state
    robot_trajectories = []
    trajectories_poses = []
    print("move_to_goal")
    id_list_duplicates = []
    for i in range(len(path_msg.poses)):
        id_list_duplicates.append(path_msg.poses[i].header.frame_id)
    id_list = list(dict.fromkeys(id_list_duplicates))
    print("Received " + str(len(id_list_duplicates)) + " grasps")
    print("id_list " + str(id_list))

    #print("Sampling all the grasps into sub-lists according to their id")
    for id in id_list:
        print("Calculating the trajectory for id " + str(id))
        pose_array_msg = geometry_msgs.msg.PoseArray()
        pose_array_msg.header.frame_id = id

        for i in range(len(path_msg.poses)):
            if path_msg.poses[i].header.frame_id == id:
                pose_array_msg.poses.append(path_msg.poses[i].pose)

        pose_array_msg.header.stamp = rospy.Time.now()
        poses_length = len(pose_array_msg.poses)
        end_index = poses_length - 2
        #print("poses_msg " + str(poses_msg))

        for i in range(0, len(pose_array_msg.poses), 2):
            success_flag = False
            poses_list = [ pose_array_msg.poses[i], pose_array_msg.poses[i+1] ]
            print("i " + str(i) + " lenght " + str(len(pose_array_msg.poses)))
            isFeasable, robot_states = validate_poses(poses_list)

            if isFeasable == True:
                print("Is Feasable. Planning Start-To-Waypoint")
                joint_states_at_waypoint = list(robot_states[0].solution.joint_state.position)
                joint_values_at_waypoint = copy.deepcopy(joint_states_at_waypoint[2:8])
                move_group.set_start_state(ready_state)
                move_group.set_joint_value_target(joint_values_at_waypoint)
                plan_waypoint = move_group.plan()

                if len(plan_waypoint.joint_trajectory.points) > 0:
                    print("Planning Waypoint-To-Goal")
                    joint_states_at_goal = list(robot_states[1].solution.joint_state.position)
                    joint_values_at_goal = copy.deepcopy(joint_states_at_goal[2:8])
                    move_group.set_start_state(robot_states[0].solution)
                    move_group.set_joint_value_target(joint_values_at_goal)
                    plan_goal = move_group.plan()
                    #move_group.set_start_state(ready_state)

                    if len(plan_goal.joint_trajectory.points) > 0:
                        print("Planning Goal-To-Waypoint")
                        move_group.set_start_state(robot_states[1].solution)
                        move_group.set_joint_value_target(joint_values_at_waypoint)
                        plan_waypoint_back = move_group.plan()

                        if len(plan_waypoint_back.joint_trajectory.points) > 0:
                            print("Found a valid plan")
                            print("---------------------------------------")
                            print("---------------------------------------")
                            waypoint_msg = geometry_msgs.msg.PoseStamped()
                            waypoint_msg.header.frame_id = id
                            waypoint_msg.header.stamp = rospy.Time.now()
                            waypoint_msg.pose = poses_list[0]
                            trajectories_poses.append(waypoint_msg)

                            goal_msg = geometry_msgs.msg.PoseStamped()
                            goal_msg.header.frame_id = id
                            goal_msg.header.stamp = rospy.Time.now()
                            goal_msg.pose = poses_list[1]
                            trajectories_poses.append(goal_msg)

                            plan_waypoint.joint_trajectory.header.frame_id = id
                            plan_goal.joint_trajectory.header.frame_id = id
                            plan_waypoint_back.joint_trajectory.header.frame_id = id
                            robot_trajectories.append(plan_waypoint)
                            robot_trajectories.append(plan_goal)
                            robot_trajectories.append(plan_waypoint_back)
                            break
            else:
                print("Not Feasable")
                if i == end_index:
                    print("None of the suggested grasps were valid")
                    print("---------------------------------------")
                    print("---------------------------------------")

    return trajectories_poses, robot_trajectories


def handle_get_trajectories(req):
    poses_array_msg = geometry_msgs.msg.PoseArray()
    poses_array_msg.header.frame_id = "world"
    poses_array_msg.header.stamp = rospy.Time.now()
    for i in range(len(req.grasps.poses)):
        poses_array_msg.poses.append(req.grasps.poses[i].pose)
    poses_pub.publish(poses_array_msg)

    print("handle_get_trajectories")
    if req.grasps.header.frame_id != "world":
        path_msg = transform_frame(req.grasps)
        trajectories_poses, robot_trajectories = move_to_goal(path_msg)
    else:
        trajectories_poses, robot_trajectories = move_to_goal(req.grasps)

    response_trajectories = moveit_scripts.msg.RobotTrajectories()
    response_trajectories.header.frame_id = "world"
    response_trajectories.header.stamp = rospy.Time.now()
    response_trajectories.trajectories = copy.deepcopy(robot_trajectories)

    response_poses = nav_msgs.msg.Path()
    response_poses.header.frame_id = "world"
    response_poses.header.stamp = rospy.Time.now()
    response_poses.poses = copy.deepcopy(trajectories_poses)

    response = GetTrajectoriesResponse()
    response.trajectories = response_trajectories
    response.trajectories_poses = response_poses

    #print("Response \n" +str(response))

    file = open("variable.txt", "w")
    str_dictionary = repr(response)
    file.write(str_dictionary)
    file.close()

    return response



if __name__ == '__main__':
    rospy.init_node('trajectory_server_ik', anonymous=True)
    poses_pub = rospy.Publisher('poses_to_reach', PoseArray, queue_size=10)

    replan_attempts = rospy.get_param("/replanning_attempts")
    abort_attempts = rospy.get_param("/abort_attempts")
    if rospy.get_param("/allow_Cartesian_planning") == False:
        success_fraction = 1.1
    else:
        success_fraction = 1.0

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.allow_replanning(rospy.get_param("/allow_replanning"))
    move_group.set_max_acceleration_scaling_factor(rospy.get_param("/acc_scaling"))
    move_group.set_max_velocity_scaling_factor(rospy.get_param("/vel_scaling"))
    move_group.set_planning_time(rospy.get_param("/planning_time"))
    move_group.set_num_planning_attempts(rospy.get_param("/planning_attempts"))

    rospy.wait_for_service('compute_ik')
    calculate_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

    trajectory_server = rospy.Service('get_trajectories', GetTrajectories, handle_get_trajectories)

    print("Checking if in READY pose")
    #print("Current state " + str(robot.get_current_state()))
    move_group.set_named_target("ready")
    plan_ready = move_group.plan()
    if len(plan_ready.joint_trajectory.points) > 0:
        print("Not in READY pose, moving the robot")
        move_group.execute(plan_ready, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        print("Done")
    #print("Current state after moving" + str(robot.get_current_state()))

    ready_state = robot.get_current_state()

    print("trajectory server is ready")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
