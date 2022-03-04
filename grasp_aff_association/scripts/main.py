#!/usr/bin/env python3

import sys
import numpy as np
import copy
import math
import cv2
import open3d as o3d

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import sensor_msgs.point_cloud2 as pc2

# ROB9
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from grasp_service.client import GraspingGeneratorClient
import rob9Utils.transformations as transform
from rob9Utils.visualize import visualizeGrasps6DOF, viz_boundingbox, viz_oriented_boundingbox, vizualizeLine
from grasp_aff_association.srv import *
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp
from rob9.srv import graspGroupSrv, graspGroupSrvResponse

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
    v1 = transform.unitVector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return transform.quaternionMultiply(
        transform.quaternionMultiply(q1, q2),
        transform.quaternionConjugate(q1)
    )[:3]

def fuse_grasp_affordance_6DOF(points, grasps_data, visualize=False, search_dist=0.1, vec_length=0.08, steps=20):
    """ input:  points - set of 3D points either as list or arrya
                grasp_data - todo
                search_distance - given in meters
        output: pointcloud_grasp - list of rob9 grasps for given point cloud
    """

    points = np.array(points)
    VISUALIZE = False

    # only perform grasp affordance association if mask has any points
    print(points.shape)
    if points.shape[0] > 15:

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
        grasp_far_viz = []
        grasp_close_viz = []

        for grasp in grasps_data:
            near = False
            if search_min[0] < grasp.position.x < search_max[0]:
                if search_min[1] < grasp.position.y < search_max[1]:
                    if search_min[2] < grasp.position.z < search_max[2]:
                        grasp_nearby.append(grasp)
                        grasp_nearby_pos.append(grasp.position.getVector(format="row"))
                        near = True
                        if VISUALIZE:
                            g_near = copy.deepcopy(grasp)
                            g_near.score = 1.0
                            grasp_close_viz.append(g_near)
            if VISUALIZE:
                if near == False:
                    g_far = copy.deepcopy(grasp)
                    g_far.score = 0.0
                    grasp_far_viz.append(g_far)

        if VISUALIZE:
            graspNear = GraspGroup(grasps = grasp_close_viz)
            graspFar = GraspGroup(grasps = grasp_far_viz)
            o3d_bbox = viz_boundingbox(search_min, search_max)
            grasp_viz_near = visualizeGrasps6DOF(cloud, graspNear)
            grasp_viz_far = visualizeGrasps6DOF(cloud, graspFar)
            o3d.visualization.draw_geometries([cloud, o3d_bbox, *grasp_viz_near, *grasp_viz_far])



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
        viz_one = True
        for i, point_list in enumerate(interpol_points):
            point_list = np.array(point_list)
            points_outside_bbox = inside_cube_test(point_list, box_corners)  # Returns point indecies outside box

            if len(points_outside_bbox) < (point_list.shape[0]):
                pointcloud_grasp.append(grasp_nearby[i])

                if VISUALIZE:
                    if viz_one:
                        vizLine = vizualizeLine(point_list[0], point_list[-1])
                        gExtrapolation = GraspGroup(grasps = [grasp_nearby[i]])
                        gExtrapolation_viz = visualizeGrasps6DOF(cloud, gExtrapolation)
                        o3d_bbox = viz_boundingbox(search_min, search_max)
                        oriented_bbox_viz = viz_oriented_boundingbox(box_corners)
                        o3d.visualization.draw_geometries([cloud, *gExtrapolation_viz, oriented_bbox_viz, vizLine])
                        viz_one = False

        return pointcloud_grasp
    return []

def handle_get_grasps(req):
    global rate

    VISUALIZE = False
    RERUN_AFFORDANCE = True

    no_objects = int(req.masks.layout.dim[0].size / 10)
    masks = np.asarray(req.masks.data).reshape((no_objects, int(req.masks.layout.dim[0].size / no_objects), req.masks.layout.dim[1].size, req.masks.layout.dim[2].size))
    masks = masks.astype(np.uint8)
    objects = np.asarray(req.objects.data)

    #graspData = GraspGroup(grasps = req.grasps)
    graspData = GraspGroup().fromGraspGroupSrv(req)

    rows = int(req.cloud_uv.layout.dim[0].size / req.cloud_uv.layout.dim[1].size)
    cols = int(req.cloud_uv.layout.dim[1].size)

    cloud_uv = np.array(req.cloud_uv.data).astype(int)
    cloud_uv = np.reshape(cloud_uv, (rows, cols))

    # Get cloud data from ros_cloud
    field_names = [field.name for field in req.cloud.fields]
    cloud_data = list(pc2.read_points(req.cloud, skip_nans=True, field_names = field_names))

    xyz = [(x, y, z) for x, y, z in cloud_data ] # get xyz
    cloud = np.array(xyz)

    if not rospy.is_shutdown():

        pub = rospy.Publisher("/Daniel", Path, queue_size=1)

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

                associated_grasps_list = []
                associated_grasps_list = fuse_grasp_affordance_6DOF(cloud_masked, graspData, visualize=False)
                if len(associated_grasps_list) > 0:
                    associated_grasps = GraspGroup(grasps = [])
                    associated_grasps = GraspGroup(grasps = copy.deepcopy(associated_grasps_list))
                    print(objects[i], affordance_id)

                    associated_grasps.setToolId(int(objects[i]))
                    associated_grasps.setAffordanceID(affordance_id)
                    associated_grasps.setObjectInstance(obj_instance)

                    graspObjects = graspObjects.combine(associated_grasps)

            print('Nr. of grasps found: ' + str(len(graspObjects.getGraspsByInstance(obj_instance))) + '  For object class: ' + str(objects[i]))

            obj_instance += 1

        #graspObjects = GraspGroup(grasps = graspObjects.getgraspsByAffordanceLabel(7))

        camera_frame = "ptu_camera_color_optical_frame"
        if req.demo.data:
            camera_frame = "ptu_camera_color_optical_frame_real"

        graspObjects.setFrameId(camera_frame)
        print("Sending...")

        return graspObjects.toGraspGroupSrv()

def main():
    global rate
    print("Setting up grasp affordance association service...")

    rospy.init_node('grasp_affordance_association', anonymous=True)
    grasp_server = rospy.Service('/grasp_affordance_association/associate', graspGroupSrv, handle_get_grasps)

    rate = rospy.Rate(5)

    print("Grasp affordance association service is ready!")

    rospy.spin()

if __name__ == "__main__":

    main()
