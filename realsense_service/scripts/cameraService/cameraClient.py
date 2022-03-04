import sys
import rospy
from realsense_service.srv import *
import numpy as np
import cv2
import sensor_msgs.point_cloud2 as pc2

class CameraClient(object):
    """docstring for CameraClient."""

    def __init__(self, type="realsenseD435"):
        self.type = type
        self.rgb = 0
        self.depth = 0
        self.uv = 0
        self.pointcloud = 0
        self.pointcloudColor = 0

        if self.type == "realsenseD435":
            self.baseService = "/sensors/realsense"
        else:
            raise Exception("Invalid type")

        self.serviceNameCapture = self.baseService + "/capture"
        self.serviceNameRGB = self.baseService + "/rgb"
        self.serviceNameDepth = self.baseService + "/depth"
        self.serviceNameUV = self.baseService + "/pointcloud/static/uv"
        self.serviceNamePointcloud = self.baseService + "/pointcloud/static"

    def captureNewScene(self):
        """ Tells the camera service to update the static data """

        rospy.wait_for_service(self.serviceNameCapture)
        captureService = rospy.ServiceProxy(self.serviceNameCapture, capture)
        msg = capture()
        msg.data = True
        response = captureService(msg)

    def getRGB(self):
        """ Sets the self.rgb to current static rgb captured by camera """

        rospy.wait_for_service(self.serviceNameRGB)
        rgbService = rospy.ServiceProxy(self.serviceNameRGB, rgb)
        msg = rgb()
        msg.data = True
        response = rgbService(msg)
        img = np.frombuffer(response.img.data, dtype=np.uint8).reshape(response.img.height, response.img.width, -1)
        self.rgb = img
        return self.rgb


    def getDepth(self):
        """ Sets the self.depth to current static depth image captured by
        camera """

        rospy.wait_for_service(self.serviceNameDepth)
        depthService = rospy.ServiceProxy(self.serviceNameDepth, depth)
        msg = depth()
        msg.data = True
        response = depthService(msg)
        img = np.frombuffer(response.img.data, dtype=np.float16).reshape(response.img.height, response.img.width, -1)
        self.depth = img
        return self.depth

    def getUvStatic(self):
        """ Sets the self.uv to current static uv coordinates for translation
        from pixel coordinates to point cloud coordinates """

        rospy.wait_for_service(self.serviceNameUV)
        uvStaticService = rospy.ServiceProxy(self.serviceNameUV, uvSrv)
        msg = uvSrv()

        msg.data = True
        response = uvStaticService(msg)
        rows = int(response.uv.layout.dim[0].size / response.uv.layout.dim[1].size)
        cols = int(response.uv.layout.dim[1].size)

        uvStatic = np.array(response.uv.data).astype(int)
        uvStatic = np.reshape(uvStatic, (rows, cols))
        self.uv = uvStatic
        return self.uv

    def getPointCloudStatic(self):
        """ sets self.pointcloud to the current static point cloud with geometry
        only """

        rospy.wait_for_service(self.serviceNamePointcloud)
        pointcloudStaticService = rospy.ServiceProxy(self.serviceNamePointcloud, pointcloud)

        msg = pointcloud()
        msg.data = True
        response = pointcloudStaticService(msg)

        # Get cloud data from ros_cloud
        field_names = [field.name for field in response.pc.fields]
        cloud_data = list(pc2.read_points(response.pc, skip_nans=True, field_names = field_names))

        # Check empty
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        xyz = [(x, y, z) for x, y, z in cloud_data ] # get xyz
        xyz = np.array(xyz)
        self.pointcloud = xyz

        # get colors
        color = np.asarray(response.color.data)
        color = np.reshape(color, (-1,3)) / 255
        self.pointcloudColor = np.flip(color, axis=1)

        return self.pointcloud, self.pointcloudColor
