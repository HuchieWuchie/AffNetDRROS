#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import std_msgs.msg
from std_msgs.msg import String, Bool

from rob9.srv import moveitMoveToNamedSrv, moveitMoveToNamedSrvResponse
from rob9.srv import moveitExecuteSrv, moveitExecuteSrvResponse
from rob9.srv import moveitRobotStateSrv, moveitRobotStateSrvResponse

def moveToNamed(name):

    if not len(name):
        print("ERROR: Specify a named pose")
        return 0

    rospy.wait_for_service("/rob9/moveit/move_to_named")
    tf2Service = rospy.ServiceProxy("/rob9/moveit/move_to_named", moveitMoveToNamedSrv)

    msg = moveitMoveToNamedSrv()
    msg.data = name

    success = tf2Service(msg)

    return success

def execute(plan):

    rospy.wait_for_service("/rob9/moveit/execute")
    tf2Service = rospy.ServiceProxy("/rob9/moveit/execute", moveitExecuteSrv)

    msg = moveitExecuteSrv()
    msg = plan

    success = tf2Service(msg)
    print(success)

    return success

def getCurrentState():

    rospy.wait_for_service("/rob9/moveit/getRobotState")
    tf2Service = rospy.ServiceProxy("/rob9/moveit/getRobotState", moveitRobotStateSrv)

    msg = moveitRobotStateSrv()
    msg.data = True

    state = tf2Service(msg).state

    return state
