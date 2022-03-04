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


# Make points ready to visualize in open3d pointcloud with color
def viz_color_pointcloud(points, color):
    colors = [color for i in range(len(points))]
    points = np.array(points)
    pc_viz = o3d.geometry.PointCloud()
    pc_viz.colors = o3d.utility.Vector3dVector(colors)
    pc_viz.points = o3d.utility.Vector3dVector(points.astype(np.float32))
    return pc_viz


# Bound selected points and visualize box in open3d
def viz_boundingbox(point_min, point_max):

    xmin = point_min[0]
    xmax = point_max[0]
    ymin = point_min[1]
    ymax = point_max[1]
    zmin = point_min[2]
    zmax = point_max[2]

    points_to_viz = [[xmin, ymin, zmin], [xmax, ymin, zmin], [xmin, ymax, zmin], [xmax, ymax, zmin],
                      [xmin, ymin, zmax], [xmax, ymin, zmax],
                      [xmin, ymax, zmax], [xmax, ymax, zmax]]
    lines_to_viz = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
             [0, 4], [1, 5], [2, 6], [3, 7]]

    colors = [[0, 0, 0] for i in range(len(lines_to_viz))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points_to_viz)
    line_set.lines = o3d.utility.Vector2iVector(lines_to_viz)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


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


def fuse_grasp_affordance(cloud_masked, grasps_data, visualize=False, search_dist=0.1, vec_length=0.08, steps=20):
    # vizualize and remove outliers
    viz_cloud = viz_color_pointcloud(cloud_masked, [1, 0, 0])
    viz_cloud, _ = viz_cloud.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

    cloud_masked = np.array(cloud_masked)
    #print(cloud_masked)
    #print(cloud_masked.shape)
    if cloud_masked.shape[0] > 4:

        bbox3d = o3d.geometry.OrientedBoundingBox.create_from_points(points=viz_cloud.points)
        box_corners = bbox3d.get_box_points()

        # Find min and max points from exstracted pointcloud, could check for outliers
        cloud_masked = np.array(cloud_masked)
        point_min = [np.min(cloud_masked[:, 0]), np.min(cloud_masked[:, 1]), np.min(cloud_masked[:, 2])]
        point_max = [np.max(cloud_masked[:, 0]), np.max(cloud_masked[:, 1]), np.max(cloud_masked[:, 2])]
        # Visualize bbox around point cloud
        bbox_cloud = viz_boundingbox(point_min, point_max)
        # print('Point min and max: ')
        # print(point_min)
        # print(point_max)

        # Find grasps close to area of interestgraspnet_msg = Bool()
        search_max = [x + search_dist for x in point_max]
        search_min = [x - search_dist for x in point_min]
        # Visualize bbox around search area for nearby grasps
        bbox_search = viz_boundingbox(search_min, search_max)
        # print('Min and max search area for nearby grasps: ')
        # print(search_min)
        # print(search_max)

        # Find nearby grasps
        grasp_nearby = []
        grasp_nearby_pos = []
        for grasp_pose in grasps_data:
            pos = [grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z]
            if search_min[0] < pos[0] < search_max[0]:
                if search_min[1] < pos[1] < search_max[1]:
                    if search_min[2] < pos[2] < search_max[2]:
                        grasp_nearby.append(grasp_pose)
                        grasp_nearby_pos.append(pos)

        # Create and vizualize direction vectors
        viz_grasp_dir = o3d.geometry.LineSet()
        lines_to_viz = []
        points_to_viz = []
        end_points = []
        for i, grasp in enumerate(grasp_nearby):
            point = [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z]
            quat = [grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z,
                    grasp.pose.orientation.w]

            vec = [1, 0, 0]
            direction = qv_mult(quat, vec)
            length = np.linalg.norm(direction)
            direction = direction / length * vec_length
            point2 = [point[0]+direction[0], point[1]+direction[1], point[2]+direction[2]]

            end_points.append(point2)
            points_to_viz.append(point)
            points_to_viz.append(point2)
            lines_to_viz.append([0 + 2 * i, 1 + 2 * i])

        colors = [[1, 0, 0] for i in range(len(lines_to_viz))]
        viz_grasp_dir.points = o3d.utility.Vector3dVector(points_to_viz)
        viz_grasp_dir.lines = o3d.utility.Vector2iVector(lines_to_viz)
        viz_grasp_dir.colors = o3d.utility.Vector3dVector(colors)

        # Visualize grasp points
        viz_grasp_points = viz_color_pointcloud(grasp_nearby_pos, color=[0, 0, 1])

        # Check which grasps point at desired object/affordance
        interpol_points = []
        for point, end_point in zip(grasp_nearby_pos, end_points):
            interpol = []
            x_diff = (point[0] - end_point[0]) / steps
            y_diff = (point[1] - end_point[1]) / steps
            z_diff = (point[2] - end_point[2]) / steps
            for i in range(1, steps + 1):
                interpol.append([point[0] - x_diff * i, point[1] - y_diff * i, point[2] - z_diff * i])
            interpol_points.append(interpol)
        # print('Interpolation points: ')
        # print(interpol_points)

        # Find points in bbox
        good_grasp = []
        interpol_viz_true = []
        interpol_viz_false = []
        for i, point_list in enumerate(interpol_points):
            grasp_intersects = False
            point_list = np.array(point_list)
            outside_bbox = inside_cube_test(point_list, box_corners)  # Returns point indecies outside box

            for idx in range(len(point_list)):
                in_bbox = True
                for index in outside_bbox:
                    if idx == index:
                        in_bbox = False

                if in_bbox:
                    interpol_viz_true.append(point_list[idx])
                    grasp_intersects = True
                else:
                    interpol_viz_false.append(point_list[idx])
            if grasp_intersects:
                good_grasp.append(grasp_nearby[i])

        # Visualize points from grasp contained in bbox
        viz_interpol_true = viz_color_pointcloud(interpol_viz_true, color=[0, 1, 0])
        # Visualize points from grasp not contained in bbox
        viz_interpol_false = viz_color_pointcloud(interpol_viz_false, color=[1, 0, 0])

        # Visualize
        if visualize:
            o3d.visualization.draw_geometries([viz_cloud, bbox3d, bbox_search, viz_grasp_dir, viz_grasp_points, viz_interpol_true, viz_interpol_false])

        #print(good_grasp)
        #print(type(good_grasp))
        return good_grasp
    else:
        return []

def qv_mult(q1, v1):
    v1 = unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[:3]


def handle_get_grasps(req):
    global tf_buffer, tf_listener, rate

    if not rospy.is_shutdown():

        # initialize the camera client
        cam = CameraClient()
        pub = rospy.Publisher("/Daniel", Path, queue_size=1)

        print('Capturing new scene...')
        cam.captureNewScene()

        # Get point cloud and uv indexes for translating from image to point
        # cloud.
        cloud, _ = cam.getPointCloudStatic()
        cloud_idx = cam.getUvStatic()

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

        print('Getting affordance results...')
        affClient = AffordanceClient()
        #   affClient.start(GPU=False)
        #_ = affClient.run(CONF_THRESHOLD = 0.3)

        _, _ = affClient.getAffordanceResult()
        affClient.processMasks(conf_threshold = 40, erode_kernel = (7,7))

        masks = affClient.masks
        objects = affClient.objects
        """
        print('Waiting for grasps from graspnet...')
        while not rospy.is_shutdown():
            if cloud is None:
                print("Waiting for point cloud...")
            if cloud_idx is None:
                print("Waiting for uv coordinates...")
            if graspData is None:
                print("Waiting for grasp service...")
            if masks is None or objects is None:
                print("Waiting for affordance service...")
            if masks is not None and objects is not None and new_grasps is not None and cloud_idx is not None and cloud is not None:
                print("Recieved everything, proceeding to grasp affordance association...")
                break
            rate.sleep()

        """
        print(cloud_idx)
        # run through all objects found by affordance net
        graspObjects = []
        for obj_idx, mask in enumerate(masks):

            # Merge masks and subtract unwanted masks
            mask_ids_to_add = [1, 2, 6, 7, 8]
            mask_full = merge_masks(mask, mask_ids_to_add, visualize=False)

            # Loop cloud indexes in mask and extract masked pointcloud
            cloud_masked = []
            for count, idx in enumerate(cloud_idx):
                if mask_full[idx[0]][idx[1]] != 0:
                    cloud_masked.append(cloud[count])
            cloud_masked = np.array(cloud_masked)
            graspObject = fuse_grasp_affordance(cloud_masked, graspData, visualize=False)

            # Set header.seq to object_id
            for grasp in graspObject:
                grasp.header.frame_id = str(objects[obj_idx])
                grasp.header.seq = objects[obj_idx]
                graspObjects.append(grasp)

            print('Nr. of grasps found: ' + str(len(graspObject)) + '  For object class: ' + str(objects[obj_idx]))


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
            #grasp.header.frame_id = camera_frame

            graspCamera = copy.deepcopy(grasp)
            waypointCamera = copy.deepcopy(grasp)
            graspCamera.header.frame_id = camera_frame
            waypointCamera.header.frame_id = camera_frame

            # computing waypoint in camera frame
            quaternion = (
                graspCamera.pose.orientation.x,
                graspCamera.pose.orientation.y,
                graspCamera.pose.orientation.z,
                graspCamera.pose.orientation.w)

            rotMat = quaternion_matrix(quaternion)[:3,:3]
            offset = np.array([[0.10], [0.0], [0.0]])
            offset = np.transpose(np.matmul(rotMat, offset))[0]

            waypointCamera.pose.position.x += -offset[0]
            waypointCamera.pose.position.y += -offset[1]
            waypointCamera.pose.position.z += -offset[2]

            # computing waypoint and grasp in world frame

            waypointWorld = transform.transformToFrame(tf_buffer, waypointCamera, camera_frame, world_frame)
            graspWorld = transform.transformToFrame(tf_buffer, graspCamera, camera_frame, world_frame)

            # computing local cartesian coordinates
            x = waypointWorld.pose.position.x - graspWorld.pose.position.x
            y = waypointWorld.pose.position.y - graspWorld.pose.position.y
            z = waypointWorld.pose.position.z - graspWorld.pose.position.z

            # computing spherical coordinates
            r, polarAngle, azimuthAngle = transform.cartesianToSpherical(x, y, z)

            # Evaluating angle limits
            azimuthAngleLimit = [-1*math.pi, 1*math.pi]
            polarAngleLimit = [0, 0.35*math.pi]

            if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
                if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:
                    waypointWorld.header.stamp = rospy.Time.now()
                    graspWorld.header.stamp = rospy.Time.now()

                    # overwrite frame_id's to be object id.
                    graspWorld.header.frame_id = grasp.header.frame_id
                    waypointWorld.header.frame_id = grasp.header.frame_id

                    waypoints.append(waypointWorld)
                    grasps.append(graspWorld)

        if len(grasps) == 0 or len(waypoints) == 0:
            print("Could not find grasp with appropriate angle")
            grasp_msg = nav_msgs.msg.Path()
            return grasps_msg # in case no grasps can be found, return empty message

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

        grasps_msg = nav_msgs.msg.Path()
        grasps_msg.header.frame_id = "world"
        grasps_msg.header.stamp = rospy.Time.now()
        for i in range(len(grasps)):
            grasps_msg.poses.append(waypoints[i])
            grasps_msg.poses[-1].header.frame_id = str(grasps[i].header.frame_id)
            grasps_msg.poses.append(grasps[i])
            grasps_msg.poses[-1].header.frame_id = str(grasps[i].header.frame_id)
        print("Sent grasps")


        rospy.sleep(1)
        pub.publish(grasps_msg)
        return grasps_msg

def main():
    global tf_buffer, tf_listener, rate
    print("Setting up grasp affordance association service...")

    rospy.init_node('grasp_affordance_association', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    grasp_server = rospy.Service('get_grasps', GetGrasps, handle_get_grasps)

    rate = rospy.Rate(5)

    print("Grasp affordance association service is ready!")
    print("v2")

    rospy.spin()

if __name__ == "__main__":

    main()
