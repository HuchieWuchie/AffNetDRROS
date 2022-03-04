#!/usr/bin/env python3
import rospy
#from grasp_generator.srv import *
import numpy as np
import cv2
import open3d as o3d
from cameraService.cameraClient import CameraClient
from grasp_service.client import GraspingGeneratorClient
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.visualize import visualizeGrasps6DOF

if __name__ == "__main__":
    print("Usage example of client server service")

    graspClient = GraspingGeneratorClient()

    # if you want to change the default grasping settings do so before starting
    # the network or running the grasping generator

    collision_thresh = 0.01 # Collision threshold in collision detection
    num_view = 300 # View number
    score_thresh = 0.1 # Remove every grasp with scores less than threshold
    voxel_size = 0.2

    graspClient.setSettings(collision_thresh, num_view, score_thresh, voxel_size)

    # Load the network with GPU (True or False) or CPU
    graspClient.start(GPU=True)

    graspGroup = graspClient.getGrasps()

    print(graspGroup.grasps[0])

    print("I got ", len(graspGroup.grasps), " grasps")

    graspGroup.thresholdByScore(0.5)

    print("I got ", len(graspGroup.grasps), " grasps after thresholding")

    cam = CameraClient()

    cloud, cloudColor = cam.getPointCloudStatic()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd.colors = o3d.utility.Vector3dVector(cloudColor)

    visualizeGrasps6DOF(pcd, graspGroup)

    #graspClient.visualizeGrasps(num_to_visualize = 0) # num_to_visualize = 0 visualizes all
    #cloud, rgb = cam.getPointCloudStatic()
