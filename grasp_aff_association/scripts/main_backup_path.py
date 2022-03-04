#!/usr/bin/env python3

import sys
import numpy as np
import copy
import math
import cv2
import open3d as o3d

import rospy
from nav_msgs.msg import Path
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply, quaternion_conjugate, unit_vector

# ROB9
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from grasp_service.client import GraspingGeneratorClient
import rob9Utils.transformations as transform
from grasp_aff_association.srv import *
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp

# Merge list of masks, return as one greyscale image mask
def merge_masks(masks, ids=[], visualize=False):
    # loop here for all objects

    mask_full = np.zeros((masks.shape[1], masks.shape[2])).astype(np.uint8)
    for i in range(1, masks.shape[0]):
        if i in ids:
            m = np.zeros((masks.shape[1], masks.shape[2])).astype(np.uint8)
            m[masks[i] > 1] = masks[i, masks[i] > 1]
            mask_full[m > 0] = m[m>0]

    if visualize:
        cv2.imshow('Mask', mask_full)
        cv2.waitKey(0)

    return mask_full

def inside_cube_test(points , cube3d):
    """
    cube3d  =  numpy array of the shape (8,3) with coordinates in the clockwise order. first the bottom plane is considered then the top one.
    points = array of points with shape (N, 3).

    Returns the indices of the points array which are outside the cube3d
    modified: https://stackoverflow.com/questions/21037241/how-to-determine-a-point-is-inside-or-outside-a-cube
    """
    b1,b2,b4,t1,t3,t4,t2,b3 = cube3d

    dir1 = (t1-b1)
    size1 = np.linalg.norm(dir1)
    dir1 = dir1 / size1

    dir2 = (b2-b1)
    size2 = np.linalg.norm(dir2)
    dir2 = dir2 / size2

    dir3 = (b4-b1)
    size3 = np.linalg.norm(dir3)
    dir3 = dir3 / size3

    cube3d_center = (b1 + t3)/2.0

    dir_vec = points - cube3d_center

    res1 = np.where( (np.absolute(np.dot(dir_vec, dir1)) * 2) > size1 )[0]
    res2 = np.where( (np.absolute(np.dot(dir_vec, dir2)) * 2) > size2 )[0]
    res3 = np.where( (np.absolute(np.dot(dir_vec, dir3)) * 2) > size3 )[0]

    return list( set().union(res1, res2, res3) )

def qv_mult(q1, v1):
    v1 = unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[:3]

def fuse_grasp_affordance(points, grasps_data, visualize=False, search_dist=0.1, vec_length=0.08, steps=20):
    """ input:  points - set of 3D points either as list or arrya
                grasp_data - todo
                search_distance - given in meters
        output: pointcloud_grasp - list of rob9 grasps for given point cloud
    """

    points = np.array(points)

    # only perform grasp affordance association if mask has any points
    if points.shape[0] > 4:

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

        # Create smallest possible bounding box around the extracted point cloud
        bbox3d = o3d.geometry.OrientedBoundingBox.create_from_points(points=cloud.points)
        box_corners = bbox3d.get_box_points()

        # Find min and max points from exstracted pointcloud
        point_min = np.array([np.min(points[:, 0]), np.min(points[:, 1]), np.min(points[:, 2])])
        point_max = np.array([np.max(points[:, 0]), np.max(points[:, 1]), np.max(points[:, 2])])

        # Determine region of interest volume, where we search for associated
        # grasps
        search_max = point_max + search_dist
        search_min = point_min - search_dist

        # Find grasps inside volume of interest
        grasp_nearby = []
        grasp_nearby_pos = []

        for grasp in grasps_data:
            if search_min[0] < grasp.position.x < search_max[0]:
                if search_min[1] < grasp.position.y < search_max[1]:
                    if search_min[2] < grasp.position.z < search_max[2]:
                        grasp_nearby.append(grasp)
                        grasp_nearby_pos.append(grasp.position.getVector(format="row"))

        # Create direction vectors based on grasping orientation
        end_points = []
        for i, grasp in enumerate(grasp_nearby):
            point = grasp.position.getVector(format="row")
            quat = grasp.orientation.getVector(format="xyzw")

            unitVector = [1, 0, 0]
            directionVector = qv_mult(quat, unitVector)
            length = np.linalg.norm(directionVector)
            directionVector = directionVector / length * vec_length
            end_point = point + directionVector

            end_points.append(end_point)

        # Check which grasps point at desired affordance
        interpol_points = []
        for point, end_point in zip(grasp_nearby_pos, end_points):
            interpol = []
            for i in range(1, steps + 1):
                interpol.append(point - (((point - end_point) / steps) * i) )
            interpol_points.append(interpol)

        pointcloud_grasp = []
        for i, point_list in enumerate(interpol_points):
            point_list = np.array(point_list)
            points_outside_bbox = inside_cube_test(point_list, box_corners)  # Returns point indecies outside box

            if len(points_outside_bbox) < (point_list.shape[0]):
                pointcloud_grasp.append(grasp_nearby[i])

        return pointcloud_grasp
    return []

def handle_get_grasps(req):
    global rate

    if not rospy.is_shutdown():

        # initialize the camera client
        cam = CameraClient()
        pub = rospy.Publisher("/Daniel", Path, queue_size=1)

        print('Capturing new scene...')
        cam.captureNewScene()

        # Get point cloud and uv indexes for translating from image to point
        # cloud.
        cloud, _ = cam.getPointCloudStatic()
        cloud_uv = cam.getUvStatic()

        # Get grasps from grasp generator
        graspClient = GraspingGeneratorClient()

        collision_thresh = 0.01 # Collision threshold in collision detection
        num_view = 300 # View number
        score_thresh = 0.0 # Remove every grasp with scores less than threshold
        voxel_size = 0.2

        graspClient.setSettings(collision_thresh, num_view, score_thresh, voxel_size)

        # Load the network with GPU (True or False) or CPU
        graspClient.start(GPU=True)

        graspData = graspClient.getGrasps()
        graspData.thresholdByScore(0.3)

        print("Got ", len(graspData), " grasps, v2")

        print('Getting affordance results...')
        affClient = AffordanceClient()
        affClient.start(GPU=False)
        _ = affClient.run(CONF_THRESHOLD = 0.5)

        _, _ = affClient.getAffordanceResult()
        affClient.processMasks(conf_threshold = 40, erode_kernel = (7,7))

        masks = affClient.masks
        objects = affClient.objects

        # run through all objects found by affordance net
        graspObjects = GraspGroup(grasps = [])

        affordance_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        obj_instance = 1

        for i in range(0, masks.shape[0]):
            for affordance_id in affordance_ids:
                # Loop cloud indexes in mask and extract masked pointcloud
                cloud_masked = []
                for k, uv in enumerate(cloud_uv):
                    if masks[i, affordance_id, uv[0], uv[1]] != 0:
                        cloud_masked.append(cloud[k])
                cloud_masked = np.array(cloud_masked)

                #associated_grasps = GraspGroup(grasps = fuse_grasp_affordance(cloud_masked, graspData, visualize=False))
                associated_grasps_list = fuse_grasp_affordance(cloud_masked, graspData, visualize=False)
                if len(associated_grasps_list) > 0:
                    associated_grasps = GraspGroup(grasps = associated_grasps_list)

                    associated_grasps.setToolId(int(objects[i]))
                    associated_grasps.setAffordanceID(affordance_id)
                    associated_grasps.setObjectInstance(obj_instance)

                    graspObjects = graspObjects.combine(associated_grasps)
                    print(len(associated_grasps), len(graspObjects))

            print('Nr. of grasps found: ' + str(len(graspObjects.getGraspsByInstance(obj_instance))) + '  For object class: ' + str(objects[i]))
            obj_instance += 1
        print(len(graspObjects), " in total")

        # MOVE EVERYTHING WAYPOINT AND SPHERICAL COORDINATE RELATED TO TRAJECTORY
        # SERVER
        # Evaluating the best grasp.
        world_frame = "world"
        ee_frame = "right_ee_link"
        if req.demo.data == True:
            camera_frame = "ptu_camera_color_optical_frame_real"
        else:
            camera_frame = "ptu_camera_color_optical_frame"

        i, grasps, waypoints = 0, [], []
        for i in range(len(graspObjects)):

            grasp = graspObjects[i]
            #grasp.frame_id = camera_frame

            graspCamera = copy.deepcopy(grasp)
            waypointCamera = copy.deepcopy(grasp)
            graspCamera.frame_id = camera_frame
            waypointCamera.frame_id = camera_frame

            # computing waypoint in camera frame
            rotMat = graspCamera.getRotationMatrix()
            offset = np.array([[0.10], [0.0], [0.0]])
            offset = np.transpose(np.matmul(rotMat, offset))[0]

            waypointCamera.position.x += -offset[0]
            waypointCamera.position.y += -offset[1]
            waypointCamera.position.z += -offset[2]

            # computing waypoint and grasp in world frame

            waypointWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(waypointCamera.toPoseStampedMsg(), world_frame))
            graspWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(graspCamera.toPoseStampedMsg(), world_frame))

            # computing local cartesian coordinates
            x = waypointWorld.position.x - graspWorld.position.x
            y = waypointWorld.position.y - graspWorld.position.y
            z = waypointWorld.position.z - graspWorld.position.z

            # computing spherical coordinates
            r, polarAngle, azimuthAngle = transform.cartesianToSpherical(x, y, z)

            # Evaluating angle limits
            azimuthAngleLimit = [-1*math.pi, 1*math.pi]
            polarAngleLimit = [0, 0.35*math.pi]
            polarAngleLimit = [0, 1*math.pi]

            if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
                if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:
                    waypointWorld.frame_id = str(graspObjects[i].tool_id)
                    graspWorld.frame_id = str(graspObjects[i].tool_id)
                    waypoints.append(waypointWorld.toPoseStampedMsg())
                    grasps.append(graspWorld.toPoseStampedMsg())

        if len(grasps) == 0 or len(waypoints) == 0:
            print("Could not find grasp with appropriate angle")
            grasp_msg = nav_msgs.msg.Path()
            return grasps_msg # in case no grasps can be found, return empty message

        """
        eeWorld = tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))
        weightedSums = []

        for i in range(len(grasps)):
            deltaRPY = abs(transform.delta_orientation(grasps[i], eeWorld))
            weightedSum = 0.2*deltaRPY[0]+0.4*deltaRPY[1]+0.4*deltaRPY[2]
            weightedSums.append(weightedSum)

        weightedSums_sorted = sorted(weightedSums)
        grasps_sorted = [None]*len(grasps) # sorted according to delta orientation from current orientation of gripper
        waypoints_sorted = [None]*len(waypoints)

        for i in range(len(weightedSums)):
            num = weightedSums_sorted[i]
            index = weightedSums.index(num)
            grasps_sorted[i] = grasps[index]
            waypoints_sorted[i] = waypoints[index]

        """

        grasps_msg = nav_msgs.msg.Path()
        grasps_msg.header.frame_id = "world"
        grasps_msg.header.stamp = rospy.Time.now()
        for i in range(len(grasps)):
            grasps_msg.poses.append(waypoints[i])
            #grasps_msg.poses[-1].header.frame_id = str(grasps[i].header.frame_id)
            grasps_msg.poses.append(grasps[i])
            #grasps_msg.poses[-1].header.frame_id = str(grasps[i].header.frame_id)
        print("Sent grasps")

        rospy.sleep(1)
        pub.publish(grasps_msg)

        return grasps_msg

def main():
    global rate
    print("Setting up grasp affordance association service...")

    rospy.init_node('grasp_affordance_association', anonymous=True)
    grasp_server = rospy.Service('get_grasps', GetGrasps, handle_get_grasps)

    rate = rospy.Rate(5)

    print("Grasp affordance association service is ready!")

    rospy.spin()

if __name__ == "__main__":

    main()
