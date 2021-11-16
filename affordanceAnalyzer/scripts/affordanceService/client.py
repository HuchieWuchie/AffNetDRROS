#!/usr/bin/env python3

import sys
import rospy
from affordance_analyzer.srv import affordanceSrv, affordanceSrvResponse
import numpy as np
import cv2
import copy
from cameraService.cameraClient import CameraClient

class AffordanceClient(object):
    """docstring for AffordanceClient."""

    def __init__(self):

        self.no_objects = 0
        self.masks = None
        self.bbox = None
        self.objects = None

        self.OBJ_CLASSES = ('__background__', 'bowl', 'tvm', 'pan', 'hammer', 'knife', 'cup', 'drill', 'racket', 'spatula', 'bottle')

    def getAffordanceResult(self, CONF_THRESHOLD = 0.7):

        rospy.wait_for_service("/affordance/result")
        affordanceNetService = rospy.ServiceProxy("/affordance/result", affordanceSrv)
        msg = affordanceSrv()
        msg.data = CONF_THRESHOLD
        response = affordanceNetService(msg)

        no_objects = int(response.masks.layout.dim[0].size / 10)
        self.no_objects = no_objects
        masks = np.asarray(response.masks.data).reshape((no_objects, int(response.masks.layout.dim[0].size / no_objects), response.masks.layout.dim[1].size, response.masks.layout.dim[2].size)) #* 255
        self.masks = masks.astype(np.uint8)

        self.bbox = np.asarray(response.bbox.data).reshape((-1,4))

        self.objects = np.asarray(response.object.data)
        print('----------Affordance results---------')
        print('Shape: ' + str(self.masks.shape))
        print('Data type: ' + str(self.masks.dtype))
        print('Bounding box: ' + str(self.bbox))
        print('Objects: ' + str(self.objects))

        return self.masks, self.objects

    def visualizeBBox(self):

        if self.bbox is None:
            print("No bounding boxes to visualize")
            return 0
        if self.bbox.shape[0] < 0:
            print("No bounding boxes to visualize")
            return 0

        print("Visualizing ", self.bbox.shape[0], " bounding boxes")

        cam = CameraClient()
        img = cam.getRGB()

        for i in range(self.bbox.shape[0]):

            x1 = int(self.bbox[i][0])
            y1 = int(self.bbox[i][1])
            w = int(int(self.bbox[i][2]) - x1)
            h = int(int(self.bbox[i][3]) - y1)

            img = cv2.rectangle(img, (x1, y1), (x1+w, y1+h), (0, 0, 255), 3)
            img = cv2.putText(img, str(self.OBJ_CLASSES[self.objects[i]]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3, 2)

        cv2.imshow("BBox's", img)
        cv2.waitKey(0)

        return 1

    def processMasks(self, conf_threshold = 50, erode_kernel=(21,21)):
        if self.masks is None:
            print("No masks available to process")
            return 0
        if self.masks.shape[0] < 0:
            print("No masks available to process")
            return 0

        kernel = np.ones(erode_kernel, np.uint8)
        for i in range(self.masks.shape[0]):
            for j in range(self.masks[i].shape[0]):

                if j >= 1: # we do not treat j=0 (background)
                    mask = copy.deepcopy(self.masks[i,j])
                    m = np.zeros((mask.shape[0], mask.shape[1])).astype(np.uint8)
                    m[mask > conf_threshold] = 255
                    m = cv2.erode(m, kernel)
                    self.masks[i,j] = m
        return 1

    def visualizeMasks(self):

        if self.masks is None:
            print("No masks available to visualize")
            return 0
        if self.masks.shape[0] < 0:
            print("No masks available to visualize")
            return 0

        print("Visualizing ", self.masks.shape[0] * self.masks.shape[1], " masks for ", self.masks.shape[0], " objects.")

        cam = CameraClient()
        img = copy.deepcopy(cam.getRGB())

        colors = [(200, 0, 0), (0,200,0), (0,0,200), (25, 74, 5), (98, 7, 55),
                (0,8,77), (10,89,33), (80, 60, 40), (20,0,99), (55, 55, 25)]

        full_mask = np.zeros(img.shape).astype(np.uint8)
        for i in range(self.masks.shape[0]):
            for j in range(self.masks[i].shape[0]):
                if j >= 1:
                    m = self.masks[i][j] != 0
                    full_mask[m] = colors[j]
        img = cv2.addWeighted(img, 1.0, full_mask, 0.7, 0)

        cv2.imshow("masks", img)
        cv2.waitKey(0)

        return 1
