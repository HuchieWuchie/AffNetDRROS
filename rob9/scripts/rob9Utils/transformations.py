#!/usr/bin/env python3
import numpy as np
import math

import rospy
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Vector3
from std_msgs.msg import String

from rob9.srv import tf2TransformPoseStampedSrv, tf2TransformPoseStampedSrvResponse
from rob9.srv import tf2TransformPathSrv, tf2TransformPathSrvResponse

def transformToFrame(pose, newFrame, currentFrame = "ptu_camera_color_optical_frame"):
    """ input:  pose - geometry_msgs.PoseStamped()
                        numpy array (x, y, z)
                        numpy array (x, y, z, qx, qy, qz, qw)
                newFrame - desired frame for pose to be transformed into.
        output: transformed_pose_msg - pose in newFrame """

    if isinstance(pose, (np.ndarray, np.generic) ):
        npArr = pose

        pose = PoseStamped()
        pose.pose.position.x = npArr[0]
        pose.pose.position.y = npArr[1]
        pose.pose.position.z = npArr[2]

        if npArr.shape[0] < 4: # no orientation provided:
            pose.pose.orientation.w = 1
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0

        else:
            pose.pose.orientation.w = npArr[6]
            pose.pose.orientation.x = npArr[3]
            pose.pose.orientation.y = npArr[4]
            pose.pose.orientation.z = npArr[5]
        pose.header.frame_id = currentFrame


    pose.header.stamp = rospy.Time.now()

    rospy.wait_for_service("/tf2/transformPoseStamped")
    tf2Service = rospy.ServiceProxy("/tf2/transformPoseStamped", tf2TransformPoseStampedSrv)

    response = tf2Service(pose, String(newFrame)).data

    return response

def transformToFramePath(path, newFrame):
    """ input:  pose - nav_msgs.Path()
            newFrame - desired frame for pose to be transformed into.
    output: transformed_path_msg - path in newFrame """

    pose.header.stamp = rospy.Time.now()

    rospy.wait_for_service("/tf2/transformPath")
    tf2Service = rospy.ServiceProxy("/tf2/transformPath", tf2TransformPathSrv)

    response = tf2Service(path, String(newFrame))

    print(response)

    return response


def quatToRot(q):
    """ input:  -   q, array [x, y, z, w]
        output: -   R, matrix 3x3 rotation matrix
        https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py
    """

    x, y, z, w = q
    quaternion = [w, x, y, z]


    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < np.finfo(float).eps * 4.0:
        return np.identity(3).flatten()

    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    R = np.array(
        [
            [
                1.0 - q[2, 2] - q[3, 3],
                q[1, 2] - q[3, 0],
                q[1, 3] + q[2, 0],
                0.0,
            ],
            [
                q[1, 2] + q[3, 0],
                1.0 - q[1, 1] - q[3, 3],
                q[2, 3] - q[1, 0],
                0.0,
            ],
            [
                q[1, 3] - q[2, 0],
                q[2, 3] + q[1, 0],
                1.0 - q[1, 1] - q[2, 2],
                0.0,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    return R[:3, :3]


def cartesianToSpherical(x, y, z):
    """ input: cartesian coordinates
        output: 3 spherical coordinates """

    polar = math.atan2(math.sqrt(x**2 + y**2), z)
    azimuth = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2 + z**2)
    return r, polar, azimuth

def quaternionMultiply(q1, q2):
    """ input:  -   q1, array or list, format xyzw
                -   q2, array or list, format xyzw
        output: -   q,  array or list, format xyzw
        https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py
    """

    # confusing order but this is on purpose

    x2, y2, z2, w2 = q1
    x1, y1, z1, w1 = q2

    q = [
            -x2 * x1 - y2 * y1 - z2 * z1 + w2 * w1,
            x2 * w1 + y2 * z1 - z2 * y1 + w2 * x1,
            -x2 * z1 + y2 * w1 + z2 * x1 + w2 * y1,
            x2 * y1 - y2 * x1 + z2 * w1 + w2 * z1,
        ]

    x, y, z, w = q[1], q[2], q[3], q[0]

    return [x, y, z, w]

def quaternionConjugate(q):
    """ input:  -   q, array or list, format xyzw
        output: -   qc,  array or list, format xyzw
        https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py
    """

    x, y, z, w = q
    qc = [-x, -y, -z, w]

    return qc

def unitVector(v):
    """ input:  -   v, array or list [x, y, z]
        output: -   v_norm,  array or list [x, y, z]
        normalizes a given vector using tf2_ros """

    v = np.array(v, dtype = np.float64, copy=True)
    if v.ndim == 1:
        v /= math.sqrt(np.dot(v, v))

    return v

def quaternionFromRotation(R):
    """ Input:  R   -   3 x 3 numpy matrix or 2D list, rotation matrix
        Output: q   -   1 x 4 numpy array, quaternion (x, y, z, w)
        https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py
    """

    # symmetric matrix K
    K = np.array(
        [
            [R[0,0] - R[1,1] - R[2,2], 0.0, 0.0, 0.0],
            [R[0,1] + R[1,0], R[1,1] - R[0,0] - R[2,2], 0.0, 0.0],
            [R[0,2] + R[2,0], R[1,2] + R[2,1], R[2,2] - R[0,0] - R[1,1], 0.0],
            [R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1], R[0,0] + R[1,1] + R[2,2]],
        ]
    )
    K /= 3.0
    # quaternion is eigenvector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        numpy.negative(q, q)

    q = np.flip(q) # reverse the array to get [x, y, z, w]
    return q

def eulerFromQuaternion(q):
    pass
"""
def delta_orientation(pose1, pose2):
    # input: 2 posestamped ros message
    #    output: the difference in rotation expressed as a quaternion

    q1 = (
        pose1.pose.orientation.x,
        pose1.pose.orientation.y,
        pose1.pose.orientation.z,
        pose1.pose.orientation.w)
    rpy1 = np.asarray(euler_from_quaternion(q1)) * 180 / math.pi

    q2Inv = (
        pose2.transform.rotation.x,
        pose2.transform.rotation.y,
        pose2.transform.rotation.z,
        -pose2.transform.rotation.w)

    deltaQuaternion = quaternionMultiply(q1, q2Inv)
    #deltaRPY = np.asarray(euler_from_quaternion(deltaQuaternion)) * 180 / math.pi
    return deltaRPY
"""
if __name__ == '__main__':
    print("Start")
    R = quatToRot([0, 0, 0, 1])
    q = quatFromRotation(R)
    print("quaternion: ", q)

    q1 = [0, 0, 0, 1]
    print(quaternionConjugate(q1))

    v = [2, 5, 1]
    print(unitVector(v))

    q2 = [0, 1, 0, 0]
    q3 = [0.008, 0.23, 0.97, 0.89]
    print(quaternionMultiply(q2, q3))

    q = quaternionMultiply(q2,q3)
    print(quatToRot(q))
