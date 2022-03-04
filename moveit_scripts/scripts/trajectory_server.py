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


def get_ik(pose_msg, current_robot_state):
    rospy.wait_for_service('compute_ik')
    calculate_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

    pose_msg_stamped = geometry_msgs.msg.PoseStamped()
    pose_msg_stamped.header.frame_id = "world"
    pose_msg_stamped.header.stamp = rospy.Time.now()
    pose_msg_stamped.pose = pose_msg

    request_msg = moveit_msgs.msg.PositionIKRequest()
    request_msg.group_name = "manipulator"
    request_msg.robot_state = current_robot_state
    request_msg.avoid_collisions = False
    request_msg.pose_stamped = pose_msg_stamped
    request_msg.timeout = rospy.Duration(10.0)
    request_msg.attempts = 10
    try:
        robot_state_out = calculate_ik(request_msg)
        return robot_state_out
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


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
            """
            #print("Calculating the trajectories for id " + str(id))
            #end_index = poses_length - 2

            waypoint_msg = geometry_msgs.msg.PoseStamped()
            waypoint_msg.header.frame_id = path_msg.header.frame_id
            waypoint_msg.header.stamp = rospy.Time.now()
            waypoint_msg.pose = pose_array_msg.poses[i]

            goal_msg = geometry_msgs.msg.PoseStamped()
            goal_msg.header.frame_id = path_msg.header.frame_id
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.pose = pose_array_msg.poses[i+1]
            """

            poses_list = [ pose_array_msg.poses[i], pose_array_msg.poses[i+1] ]
            success_flag, plan = compute_trajectory(poses_list)

            if success_flag==True:
                print("Found a valid plan")
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

                if isinstance(plan, list):
                    plan[0].joint_trajectory.header.frame_id = id
                    plan[1].joint_trajectory.header.frame_id = id
                    robot_trajectories.append(plan[0])
                    robot_trajectories.append(plan[1])
                    break
                else:
                    plan.joint_trajectory.header.frame_id = id
                    robot_trajectories.append(plan)
                    break
            else:
                if i == end_index:
                    print("None of the suggested grasps were valid")
                    #plan = None
                    #robot_trajectories.append(plan)
                else:
                    #print("poses_length " + str(poses_length) + " index i " + str(i))
                    print("Cannot reach the suggested grasp. Will try different configuration")

    return trajectories_poses, robot_trajectories

def compute_trajectory(poses_list):
    success_flag = False
    print("Computing a cartesian trajectory")
    current_pose = move_group.get_current_pose()
    start_pose = current_pose.pose
    waypoint_pose = poses_list[0]
    goal_pose = poses_list[1]
    waypoints = [start_pose, waypoint_pose, goal_pose]

    # compute cartesian trajectory
    """
    start_pose = geometry_msgs.msg.Pose()
    current_pose = move_group.get_current_pose()
    start_pose = current_pose.pose

    waypoint_pose = geometry_msgs.msg.Pose()
    waypoint_pose = poses_list[0].pose

    goal_pose = geometry_msgs.msg.Pose()
    goal_pose = poses_list[1].pose
    waypoints = [start_pose, waypoint_pose, goal_pose]
    """

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    print("Cartesian plan fraction " + str(fraction))
    #fraction = 1.1

    global success_fraction
    if fraction == success_fraction:
        #send_trajectory_to_rviz(plan)
        success_flag = True
    else:
        print("Computing a joint trajectory to the waypoint")
        success_flag_waypoint, plan_waypoint = compute_start_to_waypoint(waypoint_pose)
        if success_flag_waypoint == True:
            print("Computing a joint trajectory from the waypoint to the goal")
            current_state = robot.get_current_state()
            start_state = moveit_msgs.msg.RobotState()
            start_state = get_ik(waypoint_pose, current_state)
            move_group.set_start_state(start_state)
            waypoints_goal = [waypoint_pose, goal_pose]
            plan_goal, fraction = move_group.compute_cartesian_path(waypoints_goal, 0.01, 0.0)
            move_group.set_start_state(current_state)
            if fraction == 1.0:
                print("Cartesian path for waypoint to goal")
                success_flag = True
                plan = [plan_waypoint, plan_goal]
            else:
                print("Nope")
                success_flag_goal, plan_goal = compute_waypoint_to_goal(waypoint_pose, goal_pose)
                if success_flag_goal == True:
                    print("Waypoint plan lenght is " + str(len(plan_waypoint.joint_trajectory.points)) )
                    print("Goal plan lenght is " + str(len(plan_goal.joint_trajectory.points)) )
                    success_flag = True
                    plan = [plan_waypoint, plan_goal]

    return success_flag, plan


def compute_start_to_waypoint(waypoint_pose):
    current_state = moveit_msgs.msg.RobotState()
    current_state = robot.get_current_state()
    move_group.set_start_state(current_state)
    move_group.set_pose_target(waypoint_pose)

    plan_list = []
    plan_length = []
    global replan_attempts, abort_attempts
    for i in range(replan_attempts):
        plan = move_group.plan()
        plan_list.append(plan)

        length = len(plan.joint_trajectory.points)
        if length == 0:
            length = 9999
        plan_length.append(length)

        if i == abort_attempts-1:
            counter = 0
            for j in range (abort_attempts):
                if plan_length[j] == 9999:
                    counter += 1
            if counter == abort_attempts:
                print("Quitting planning, the first five attempts failed")
                break

    min_index = plan_length.index(min(plan_length))
    plan = plan_list[min_index]
    if plan_length[min_index] == 9999:
        #plan = None
        success_flag = False
    else:
        #send_trajectory_to_rviz(plan)
        success_flag = True

    return success_flag, plan


def compute_waypoint_to_goal(waypoint_pose, goal_pose):
    current_state = moveit_msgs.msg.RobotState()
    current_state = robot.get_current_state()
    start_state = moveit_msgs.msg.RobotState()
    start_state = get_ik(waypoint_pose, current_state)
    move_group.set_start_state(start_state)

    #print("compute_waypoint_to_goal state " + str(start_state))
    move_group.set_pose_target(goal_pose)
    plan_list = []
    plan_length = []
    global replan_attempts, abort_attempts
    for i in range(replan_attempts):
        plan = move_group.plan()
        plan_list.append(plan)

        length = len(plan.joint_trajectory.points)
        if length == 0:
            length = 9999
        plan_length.append(length)

        if i == abort_attempts-1:
            counter = 0
            for j in range (abort_attempts):
                if plan_length[j] == 9999:
                    counter += 1
            if counter == abort_attempts:
                print("Quitting planning, the first five attempts failed")
                break

    #print("Plan lengths " + str(plan_length))
    min_index = plan_length.index(min(plan_length))
    #print("Min index " + str(min_index))
    plan = plan_list[min_index]
    if plan_length[min_index] == 9999:
        #plan = None
        success_flag = False
    else:
        #send_trajectory_to_rviz(plan)
        success_flag = True

    move_group.set_start_state(current_state)
    #move_group.set_start_state_to_current_state()
    return success_flag, plan


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
    rospy.init_node('moveit_subscriber', anonymous=True)
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

    trajectory_server = rospy.Service('get_trajectories', GetTrajectories, handle_get_trajectories)
    print("trajectory server is ready")


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
