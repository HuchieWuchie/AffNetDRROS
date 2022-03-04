#!/usr/bin/env python3
import rospy
from affordance_analyzer.srv import getAffordanceSrv, getAffordanceSrvResponse
import numpy as np
import cv2
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient

import open3d as o3d
import copy

if __name__ == "__main__":
    print("Usage example of client server service")
    rospy.init_node('affordance_analyzer_client', anonymous=True)

    affClient = AffordanceClient()
    cam = CameraClient()
    cam.captureNewScene()
    cloud, cloudColor = cam.getPointCloudStatic()
    cloud_uv = cam.getUvStatic()
    img = cam.getRGB()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd.colors = o3d.utility.Vector3dVector(cloudColor)
    #affClient.start(GPU=True) # GPU acceleratio True or False
    print(affClient.name)
    print("Telling affordanceNET to analyze image from realsense and return predictions.")
    success = affClient.run(img, CONF_THRESHOLD = 0.7)
    masks, objects, scores, bbox = affClient.getAffordanceResult()
    print("Got result")

    print(scores)


    masks = affClient.processMasks(conf_threshold = 230, erode_kernel=(3,3))
    affClient.masks = masks
    #width, height = img.shape[1], img.shape[0]
    #ratio = width / height
    #img = cv2.resize(img, (int(450 * ratio), 450), interpolation = cv2.INTER_AREA)
    img = affClient.visualizeMasks(img)
    img = affClient.visualizeBBox(img)
    cv2.imshow("output", img)
    cv2.waitKey(0)


    clouds_m = []
    for obj_idx, obj_id in enumerate(objects):
        print(obj_id, obj_idx)
        affordance_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        colors = [(0,0,205), (34,139,34), (192,192,128), (165, 42, 42), (128, 64, 128),
                (204, 102, 0), (184, 134, 11), (0, 153, 153), (0, 134, 141), (184, 0, 141), (184, 0, 255)]

        for affordance_id in affordance_ids:

            ## mask point cloud
            cloud_masked_points = []
            color_masked = []
            for k, uv in enumerate(cloud_uv):
                if masks[obj_idx, affordance_id, uv[0], uv[1]] == 1:
                    cloud_masked_points.append(cloud[k])
                    c = img[uv[0], uv[1]] / 255
                    #c_rgb = (c[2], c[1], c[0])
                    color_masked.append(c)

            if len(cloud_masked_points) > 20:
                pcd_m = o3d.geometry.PointCloud()
                #color_bgr = colors[affordance_id]
                #color_rgb = (color_bgr[2], color_bgr[1], color_bgr[0])
                pcd_m.points = o3d.utility.Vector3dVector(cloud_masked_points)
                pcd_m.colors = o3d.utility.Vector3dVector(color_masked)

                #pcd_m.paint_uniform_color(color_rgb)
                clouds_m.append(copy.deepcopy(pcd_m))

                o3d.visualization.draw_geometries([pcd_m])

        o3d.visualization.draw_geometries([*clouds_m])


    print("Found the following objects")
    for i in range(len(affClient.objects)):
        print(affClient.OBJ_CLASSES[affClient.objects[i]])
