#!/usr/bin/env python3
import rospy
from grasp_generator.srv import *
from std_msgs.msg import Float32, Int32
from tf.transformations import quaternion_matrix
import numpy as np
import cv2
import open3d as o3d
import copy
from cameraService.cameraClient import CameraClient

class GraspingGeneratorClient(object):
    """docstring for GraspingGeneratorClient."""

    def __init__(self):

        self.grasps = None
        self.GPU = False

        self.collision_thresh = 0.01 # Collision threshold in collision detection
        self.num_view = 300 # View number
        self.score_thresh = 0.0 # Remove every grasp with scores less than threshold
        self.voxel_size = 0.2

    def start(self, GPU=True):
        """ current docker image implementation only runs on GPU """

        self.GPU = GPU
        rospy.wait_for_service("/grasp_generator/start")
        startGraspingGeneratorService = rospy.ServiceProxy("/grasp_generator/start", startGraspingSrv)
        msg = startGraspingSrv()
        msg.data = self.GPU
        response = startGraspingGeneratorService(msg)

    def getGrasps(self):

        rospy.wait_for_service("/grasp_generator/result")
        graspGeneratorService = rospy.ServiceProxy("/grasp_generator/result", runGraspingSrv)
        msg = runGraspingSrv()
        msg.data = True
        response = graspGeneratorService(msg)

        self.grasps = response.grasps
        return self.grasps.poses

    def setSettings(self, collision_thresh = 0.01, num_view = 300, score_thresh = 0.0, voxel_size = 0.2):

        self.collision_thresh = collision_thresh # Collision threshold in collision detection
        self.num_view = num_view # View number
        self.score_thresh = score_thresh # Remove every grasp with scores less than threshold
        self.voxel_size = voxel_size

        rospy.wait_for_service("/grasp_generator/set_settings")
        graspGeneratorService = rospy.ServiceProxy("/grasp_generator/set_settings", setSettingsGraspingSrv)

        response = graspGeneratorService(Float32(collision_thresh), Int32(num_view), Float32(score_thresh), Float32(voxel_size))

    def visualizeGrasps(self, num_to_visualize=0):
        """ Visualizes grasps in a point cloud, sorry for this monster of a
            function.
            Input:  num_to_visualize - number of highest scoring grasps to viz
            Output: None """

        # reconfigure grasps from posestamepd_msgs
        grasps = []
        #print(self.grasps.poses[0].pose.position)
        for grasp in self.grasps.poses:

            x = grasp.pose.position.x
            y = grasp.pose.position.y
            z = grasp.pose.position.z
            qx = grasp.pose.orientation.x
            qy = grasp.pose.orientation.y
            qz = grasp.pose.orientation.z
            qw = grasp.pose.orientation.w
            score = float(grasp.header.frame_id)

            grasps.append([score, x, y, z, qx, qy, qz, qw])

        grasps.sort(key=lambda x:x[0], reverse=True) # sort according to score
        if len(grasps) > num_to_visualize and num_to_visualize != 0:
            grasps = grasps[:num_to_visualize]

        # gripper geometry
        width, height, depth = 0.02, 0.02, 0.02
        height=0.004
        finger_width = 0.004
        tail_length = 0.04
        depth_base = 0.02

        # make a list of open3d geometry
        o3d_gripper_geom = []
        for grasp in grasps:

            center = np.array([grasp[1], grasp[2], grasp[3]])
            R = quaternion_matrix([grasp[4], grasp[5], grasp[6], grasp[7]])[:3,:3]
            score = grasp[0]

            color_r, color_g, color_b = score, 0, 1 - score

            left = create_mesh_box(depth+depth_base+finger_width, finger_width, height)
            right = create_mesh_box(depth+depth_base+finger_width, finger_width, height)
            bottom = create_mesh_box(finger_width, width, height)
            tail = create_mesh_box(tail_length, finger_width, height)

            left_points = np.array(left.vertices)
            left_triangles = np.array(left.triangles)
            left_points[:,0] -= depth_base + finger_width
            left_points[:,1] -= width/2 + finger_width
            left_points[:,2] -= height/2

            right_points = np.array(right.vertices)
            right_triangles = np.array(right.triangles) + 8
            right_points[:,0] -= depth_base + finger_width
            right_points[:,1] += width/2
            right_points[:,2] -= height/2

            bottom_points = np.array(bottom.vertices)
            bottom_triangles = np.array(bottom.triangles) + 16
            bottom_points[:,0] -= finger_width + depth_base
            bottom_points[:,1] -= width/2
            bottom_points[:,2] -= height/2

            tail_points = np.array(tail.vertices)
            tail_triangles = np.array(tail.triangles) + 24
            tail_points[:,0] -= tail_length + finger_width + depth_base
            tail_points[:,1] -= finger_width / 2
            tail_points[:,2] -= height/2

            vertices = np.concatenate([left_points, right_points, bottom_points, tail_points], axis=0)
            vertices = np.dot(R, vertices.T).T + center
            triangles = np.concatenate([left_triangles, right_triangles, bottom_triangles, tail_triangles], axis=0)
            colors = np.array([ [color_r,color_g,color_b] for _ in range(len(vertices))])

            gripper = o3d.geometry.TriangleMesh()
            gripper.vertices = o3d.utility.Vector3dVector(vertices)
            gripper.triangles = o3d.utility.Vector3iVector(triangles)
            gripper.vertex_colors = o3d.utility.Vector3dVector(colors)

            o3d_gripper_geom.append(gripper)

        cam = CameraClient()

        cloud, cloudColor = cam.getPointCloudStatic()
        #cloud[:,2] = -cloud[:,2]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        pcd.colors = o3d.utility.Vector3dVector(cloudColor)

        o3d.visualization.draw_geometries([pcd, *o3d_gripper_geom])

        return o3d_gripper_geom

def create_mesh_box(width, height, depth, dx=0, dy=0, dz=0):
    ''' Author: chenxi-wang
    Create box instance with mesh representation.
    '''
    box = o3d.geometry.TriangleMesh()
    vertices = np.array([[0,0,0],
                         [width,0,0],
                         [0,0,depth],
                         [width,0,depth],
                         [0,height,0],
                         [width,height,0],
                         [0,height,depth],
                         [width,height,depth]])
    vertices[:,0] += dx
    vertices[:,1] += dy
    vertices[:,2] += dz
    triangles = np.array([[4,7,5],[4,6,7],[0,2,4],[2,6,4],
                          [0,1,2],[1,3,2],[1,5,7],[1,7,3],
                          [2,3,7],[2,7,6],[0,4,1],[1,4,5]])
    box.vertices = o3d.utility.Vector3dVector(vertices)
    box.triangles = o3d.utility.Vector3iVector(triangles)
    return box
