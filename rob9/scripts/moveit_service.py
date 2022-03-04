#!/usr/bin/env python
import sys

import rospy
import moveit_commander
import moveit_msgs

from rob9.srv import moveitMoveToNamedSrv, moveitMoveToNamedSrvResponse
from rob9.srv import moveitExecuteSrv, moveitExecuteSrvResponse
from rob9.srv import moveitRobotStateSrv, moveitRobotStateSrvResponse

def moveToNamed(req):

    print("Moving robot to named position: ", req.name.data)

    move_group.set_named_target(req.name.data)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    resp = moveitMoveToNamedSrvResponse()
    resp.success.data = True

    return resp

def execute(req):

    move_group.execute(req.trajectory, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    resp = moveitExecuteSrvResponse()
    resp.success.data = True

    return resp

def getCurrentState(req):

    return robot.get_current_state()

if __name__ == '__main__':

    baseServiceName = "/rob9/moveit/"

    rospy.init_node('moveit_service', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    moveToNameService = rospy.Service(baseServiceName + "move_to_named", moveitMoveToNamedSrv, moveToNamed)
    executeService = rospy.Service(baseServiceName + "execute", moveitExecuteSrv, execute)
    robotStateService = rospy.Service(baseServiceName + "getRobotState", moveitRobotStateSrv, getCurrentState)

    rospy.spin()
