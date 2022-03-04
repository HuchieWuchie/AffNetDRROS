#!/usr/bin/env python
import sys
import rospy
import std_msgs.msg
from grasp_pose.srv import *

def client():
    print("client")
    demo = std_msgs.msg.Bool()
    demo.data = True
    msg = GetGrasps()
    msg.data = True
    rospy.wait_for_service('get_grasps')
    grasps = nav_msgs.msg.Path()
    try:
        get_grasps = rospy.ServiceProxy('get_grasps', GetGrasps)
        resp = get_grasps(msg)
        print("I have received a response \n"  + str(resp.grasps))
        return resp.grasps
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node('grasp_association_client', anonymous=True)

    rate = rospy.Rate(5)
    client()
