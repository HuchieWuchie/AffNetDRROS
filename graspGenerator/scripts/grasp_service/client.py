#!/usr/bin/env python3
import rospy
from grasp_generator.srv import *
from rob9.msg import *
from rob9.srv import *
from std_msgs.msg import Float32, Int32
import numpy as np
import cv2
import open3d as o3d
import copy
from cameraService.cameraClient import CameraClient
from rob9Utils.graspGroup import GraspGroup

class GraspingGeneratorClient(object):
    """docstring for GraspingGeneratorClient."""

    def __init__(self):

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
        grasps = GraspGroup().fromGraspGroupMsg(response)

        del response

        return grasps

    def setSettings(self, collision_thresh = 0.01, num_view = 300, score_thresh = 0.0, voxel_size = 0.2):

        self.collision_thresh = collision_thresh # Collision threshold in collision detection
        self.num_view = num_view # View number
        self.score_thresh = score_thresh # Remove every grasp with scores less than threshold
        self.voxel_size = voxel_size

        rospy.wait_for_service("/grasp_generator/set_settings")
        graspGeneratorService = rospy.ServiceProxy("/grasp_generator/set_settings", setSettingsGraspingSrv)

        response = graspGeneratorService(Float32(collision_thresh), Int32(num_view), Float32(score_thresh), Float32(voxel_size))
