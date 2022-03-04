#!/usr/bin/env python3

import os
import sys
import torch
import rospy
import numpy as np
import scipy.io as scio
from ctypes import * # convert float to uint32

from nav_msgs.msg import Path
from graspnetAPI import GraspGroup
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header, Float32

from grasp_generator.srv import *
from rob9.msg import *
from rob9.srv import *
from cameraService.cameraClient import CameraClient
#from rob9Utils.graspGroup import GraspGroup as rob9GraspGroup
#from rob9Utils.grasp import Grasp as rob9Grasp

ROOT_DIR = '/graspnet/graspnet-baseline/'  # path to graspnet-baseline
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
SCRIPT_DIR = os.path.normpath(__file__ + os.sep + os.pardir)

from graspnet import GraspNet, pred_decode
from collision_detector import ModelFreeCollisionDetector

class GraspServer(object):
    """docstring for GraspServer."""

    def __init__(self):

        print('Starting...')
        rospy.init_node('grasp_generator', anonymous=True)

        self.rate = rospy.Rate(5)
        self.net = None
        self.GPU = False

        # Default values
        self.checkpoint_path = SCRIPT_DIR + '/checkpoint-rs.tar' # path to weights
        self.collision_thresh = 0.01 # Collision threshold in collision detection
        self.num_view = 300 # View number
        self.score_thresh = 0.0 # Remove every grasp with scores less than threshold
        self.voxel_size = 0.2 # Voxel Size to process point clouds before collision detection

        self.serviceRun = rospy.Service("grasp_generator/result", runGraspingSrv, self.run)
        self.serviceStart = rospy.Service("grasp_generator/start", startGraspingSrv, self.start)
        self.serviceSetSettings = rospy.Service("grasp_generator/set_settings", setSettingsGraspingSrv, self.set_settings)

    def set_settings(self, msg):

        self.collision_thresh = msg.collision_thresh.data
        self.num_view = msg.num_view.data
        self.score_thresh = msg.score_thresh.data
        self.voxel_size = msg.voxel_size.data

        print("Running grasp generator with these settings: ")
        print("Collision threshold: ", self.collision_thresh)
        print("Num veiw: ", self.num_view)
        print("Grasp score threshold: ", self.score_thresh)
        print("Voxel size: ", self.voxel_size)

        return setSettingsGraspingSrvResponse()

    def start(self, msg):

        print('Loading network...')
        self.GPU = msg.GPU.data
        self.net = self.get_net()
        print('Network loaded')
        print('Waiting for go...')

        return startGraspingSrvResponse()

    def run(self, msg):

        cam = CameraClient()

        if self.net is None:
            print("No grasping network is initialized.")
            return runGraspingSrvResponse()

        cloud, rgb = cam.getPointCloudStatic()

        while cloud is None or rgb is None:
            self.rate.sleep

        print('Processing image through graspnet...')

        end_points = self.get_and_process_data(cloud, rgb)

        gg = self.get_grasps(end_points)
        if self.collision_thresh > 0:
            gg = self.collision_detection(gg, cloud)
        gg.nms()
        gg = self.remove_grasps_under_score(gg, self.score_thresh) #  Score range between 0 and 2. Under 0.1 bad, over 0.7 good

        print("Computed grasps, now sending...")

        return self.generate_ros_message(gg)

    def get_net(self):
        # Init the model
        net = GraspNet(input_feature_dim=0, num_view=self.num_view, num_angle=12, num_depth=4,
                 cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
        device = torch.device("cuda:0" if self.GPU else "cpu")
        net.to(device)
        checkpoint = torch.load(self.checkpoint_path)
        net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch']
        print("-> loaded checkpoint %s (epoch: %d)"%(self.checkpoint_path, start_epoch))
        net.eval()

        return net


    def get_and_process_data(self, cloud, rgb):

        end_points = dict()
        cloud_sampled = torch.from_numpy(cloud[np.newaxis].astype(np.float32))
        device = torch.device("cuda:0" if self.GPU else "cpu")
        cloud_sampled = cloud_sampled.to(device)

        end_points['point_clouds'] = cloud_sampled
        end_points['cloud_colors'] = rgb

        return end_points

    def get_grasps(self, end_points):
        # Forward pass
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(gg_array)
        return gg

    def generate_ros_message(self, gg):

        grasps = gg
        seq = 0

        #graspGroup = rob9GraspGroup()
        graspGroupMsg = GraspGroupMsg()
        graspList = []

        for grap in grasps:

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = str("ptu_camera_color_optical_frame_real")
            header.seq = seq
            seq += 1

            pose = Pose()
            pose.position.x = grap.translation[0]
            pose.position.y = grap.translation[1]
            pose.position.z = grap.translation[2]

            rot = Rotation.from_matrix(grap.rotation_matrix)
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            m = GraspMsg()
            m.score.data = grap.score
            m.header = header
            m.pose = pose

            graspList.append(m)

        graspGroupMsg.grasps = graspList

        return graspGroupMsg

    def remove_grasps_under_score(self, gg, score_thresh):
        gg.sort_by_score()
        grasp_nr = 0
        for scores in gg.scores:
            if scores < score_thresh:
                gg = gg[:grasp_nr]
            grasp_nr += 1
        return gg

    def collision_detection(self, gg, cloud):
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
        gg = gg[~collision_mask]
        return gg

if __name__ == "__main__":

    graspGenerator = GraspServer()
    print("Grasp generator is ready")

    while not rospy.is_shutdown():
        rospy.spin()
