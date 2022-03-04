#!/usr/bin/env python
import sys
import rospy
#import std_msgs.msg
from std_msgs.msg import *
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
# from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply
import copy
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def run_graspnet(pub):
    print('Send start to graspnet')
    graspnet_msg = std_msgs.msg.Bool()
    graspnet_msg.data = True
    print(graspnet_msg)
    pub.publish(graspnet_msg)
    print("Done")

if __name__ == "__main__":
    rospy.init_node('test_node', anonymous=True)
    pub_graspnet = rospy.Publisher('start_graspnet', Bool, queue_size=1, latch=True)
    #rospy.sleep(5.)
    graspnet_msg = std_msgs.msg.Bool()
    graspnet_msg.data = True
    print(graspnet_msg)
    pub_graspnet.publish(graspnet_msg)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    #rospy.spin()
    #run_graspnet(pub_graspnet)
    #rospy.spin()
