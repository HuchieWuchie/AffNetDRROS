#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
# from geometry_msgs.msg import Pose
import std_msgs.msg
from std_msgs.msg import Int8
from math import pi
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf2_ros
# from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


def transform_frame(msg):
    print("Transforming message to the world frame")
    transformed_poses = geometry_msgs.msg.PoseArray()
    transformed_poses.header.frame_id = "world"
    tf_buffer.lookup_transform(msg.header.frame_id, 'world', rospy.Time.now(), rospy.Duration(1.0))
    for i in range(len(msg.poses)):
        transformed_pose_msg = geometry_msgs.msg.PoseStamped()
        transformed_pose_msg = tf_buffer.transform(msg.pose[i], "world")
        transformed_poses.poses.append(transformed_pose_msg.pose)
    transformed_poses.header.stamp = rospy.Time.now()
    move_to_goal(trasnformed_poses)


def move_to_ready():
    print("Moving to ready...")
    ready_pose_msg = geometry_msgs.msg.PoseStamped()
    ready_pose_msg.header.frame_id = "world"
    ready_pose_msg.header.stamp = rospy.Time.now()
    ready_pose_msg.pose.position.x = 0.4071
    ready_pose_msg.pose.position.y = 0.1361
    ready_pose_msg.pose.position.z = 1.6743
    ready_pose_msg.pose.orientation.x = -0.0575
    ready_pose_msg.pose.orientation.y = 0.4495
    ready_pose_msg.pose.orientation.z = 0.7546
    ready_pose_msg.pose.orientation.w = 0.4745

    success_flag, plan = compute_trajectory(ready_pose_msg)
    if success_flag == True:
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        print("Done")
    else:
        print("Cannot reach the ready pose")


def send_trajectory_to_rviz(plan):
    print("Trajectory was sent to RViZ")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


def move_to_goal(poses_msg):
    print("Validating received grasps")
    for i in range(0, len(poses_msg.poses), 2):
        waypoint_msg = geometry_msgs.msg.PoseStamped()
        waypoint_msg.header.frame_id = poses_msg.header.frame_id
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.pose = poses_msg.poses[i]
        pub_waypoint.publish(waypoint_msg)

        goal_msg = geometry_msgs.msg.PoseStamped()
        goal_msg.header.frame_id = poses_msg.header.frame_id
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = poses_msg.poses[i+1]
        pub_grasp.publish(goal_msg)

        success_flag_waypoint, plan_waypoint = compute_trajectory(waypoint_msg)
        if success_flag_waypoint == True:
            print("Found a valid plan for the waypoint")
            print("Executing the  waypoint trajectory")
            rospy.sleep(10.)
            move_group.execute(plan_waypoint, wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            print("Done. Validating the grasp")
            success_flag_goal, plan_goal = compute_trajectory(goal_msg)
            if success_flag_goal == True:
                print("Found a valid plan for the grasp")
                print("Executing the goal trajectory")
                rospy.sleep(10.)
                move_group.execute(plan_goal, wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                print("Done")
                rospy.sleep(10.)
                move_to_ready()
                break
            else:
                continue
        else:
            if i == len(poses_msg.poses):
                print("None of the suggested grasps were valid")
            continue


def execute_trajectory(plan):
    print("Executing the planned trajectory")
    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    print("Done")


def compute_trajectory(msg):
    print("Computing a cartesian trajectory")
    # compute cartesian trajectory
    start_pose = geometry_msgs.msg.Pose()
    current_pose = move_group.get_current_pose()
    start_pose = current_pose.pose
    goal_pose = geometry_msgs.msg.Pose()
    goal_pose = msg.pose
    waypoints = [start_pose, goal_pose]
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    print("Cartesian plan fraction " + str(fraction))

    if fraction == 1.0:
        send_trajectory_to_rviz(plan)
        success_flag = True
    else:
        move_group.set_pose_target(goal_pose)
        plan_list = []
        plan_length = []
        for i in range(20):
            plan = move_group.plan()
            plan_list.append(plan)
            # print(plan)
            length = len(plan.joint_trajectory.points)
            if length == 0:
                length = 9999
            plan_length.append(length)
            if i == 2:
                counter = 0
                for j in range (3):
                    if plan_length[j]==9999:
                        counter += 1
                if counter == 3:
                    print("Quitting planning, the first three attempts failed")
                    break

        #print("Plan lengths " + str(plan_length))
        min_index = plan_length.index(min(plan_length))
        #print("Min index " + str(min_index))
        plan = plan_list[min_index]
        if plan_length[min_index] == 9999:
            plan = None
            success_flag = False
        else:
            send_trajectory_to_rviz(plan)
            success_flag = True
    return success_flag, plan


def callback(msg):
    print("Callback")
    #print(msg)
    if msg.header.frame_id != "world":
        transform_frame(msg)
    else:
        move_to_goal(msg)

def reset_callback(msg):
    print("Reset callback")
    if msg.data == 1:
        global reset_gripper_msg, activate_gripper_msg, pinch_gripper_msg
        move_group.set_named_target("ready")
        plan = move_group.go(wait=True)
        move_group.stop()
        #gripper_pub.publish(reset_gripper_msg)
        #gripper_pub.publish(activate_gripper_msg)
        #gripper_pub.publish(pinch_gripper_msg)
    else:
        print("Invalid input")


if __name__ == '__main__':
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('poses_to_reach', PoseArray, callback)
    rospy.Subscriber('reset_robot', Int8, reset_callback)
    gripper_pub = rospy.Publisher('gripper_controller', Int8, queue_size=1)
    pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
    pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    """
    planning_time = move_group.get_planning_time()
    goal_orientation_tolerance = move_group.get_goal_orientation_tolerance()
    goal_position_tolerance = move_group.get_goal_position_tolerance()
    print "planning_time", planning_time
    print "goal orientation tolerance", goal_orientation_tolerance
    print "goal_position_tolerance", goal_position_tolerance
    """
    move_group.allow_replanning(True)
    move_group.set_max_acceleration_scaling_factor(0.5)
    move_group.set_max_velocity_scaling_factor(0.5)
    #move_group.set_planning_time(0.5)
    #move_group.set_num_planning_attempts(25)
    move_group.set_goal_orientation_tolerance(0.1)
    move_group.set_goal_position_tolerance(0.01)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    reset_gripper_msg = std_msgs.msg.Int8()
    reset_gripper_msg.data = 0
    activate_gripper_msg = std_msgs.msg.Int8()
    activate_gripper_msg.data = 1
    close_gripper_msg = std_msgs.msg.Int8()
    close_gripper_msg = 2
    open_gripper_msg = std_msgs.msg.Int8()
    open_gripper_msg.data = 3
    basic_gripper_msg = std_msgs.msg.Int8()
    basic_gripper_msg.data = 4
    pinch_gripper_msg = std_msgs.msg.Int8()
    pinch_gripper_msg.data = 5

    gripper_pub.publish(reset_gripper_msg)
    gripper_pub.publish(activate_gripper_msg)
    #gripper_pub.publish(pinch_gripper_msg)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
