#!/usr/bin/env python
import rospy
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs

from rob9.srv import tf2TransformPoseStampedSrv, tf2TransformPoseStampedSrvResponse
from rob9.srv import tf2TransformPathSrv, tf2TransformPathSrvResponse
#from rob9.srv import tf2getQuatSrv, tf2getQuatSrvResponse

def transformPathToFrame(req):

    newFrame = req.new_frame.data

    transformed_poses = []
    for pose in req.poses:
        pose.header.stamp = rospy.Time.now()
        transformed_pose_msg = geometry_msgs.msg.PoseStamped()
        tf_buffer.lookup_transform(pose.header.frame_id, newFrame, rospy.Time.now(), rospy.Duration(1))
        transformed_poses.append(tf_buffer.transform(pose, newFrame))

    path = Path()
    path.poses = transformed_poses

    return transformed_pose_msg


def transformToFrame(req):

    pose = req.pose
    newFrame = req.new_frame.data

    pose.header.stamp = rospy.Time.now()
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    tf_buffer.lookup_transform(pose.header.frame_id, newFrame, rospy.Time.now(), rospy.Duration(1))
    transformed_pose_msg = tf_buffer.transform(pose, newFrame)
    return transformed_pose_msg


if __name__ == '__main__':

    baseServiceName = "/tf2/"

    rospy.init_node('tf2_service', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    transformPoseStampedService = rospy.Service(baseServiceName + "transformPoseStamped", tf2TransformPoseStampedSrv, transformToFrame)
    transformPathService = rospy.Service(baseServiceName + "transformPath", tf2TransformPathSrv, transformToFrame)

    rospy.spin()
