#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import open3d as o3d
from rob9.msg import *
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp

def visualizeGrasps6DOF(pointcloud, graspGroup, color = None):
    """ input:  pointcloud - open3d pointcloud
                graspGroup - rob9Utils.graspGroup GraspGroup()
    """

    """ Visualizes grasps in a point cloud, sorry for this monster of a
        function.
        Input:  num_to_visualize - number of highest scoring grasps to viz
        Output: None """

    # gripper geometry
    width, height, depth = 0.02, 0.02, 0.02
    height=0.004
    finger_width = 0.004
    tail_length = 0.04
    depth_base = 0.02

    # make a list of open3d geometry
    o3d_gripper_geom = []
    for grasp in graspGroup.grasps:

        center = grasp.position.getVector(format="row")
        R = grasp.orientation.getRotationMatrix()
        score = grasp.score




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

        colors = None
        if color == None:
            color_r, color_g, color_b = score, 0, 1 - score
            colors = np.array([ [color_r,color_g,color_b] for _ in range(len(vertices))])
        else:
            color_r, color_g, color_b = color
            colors = np.array([ [color_r,color_g,color_b] for _ in range(len(vertices))])


        gripper = o3d.geometry.TriangleMesh()
        gripper.vertices = o3d.utility.Vector3dVector(vertices)
        gripper.triangles = o3d.utility.Vector3iVector(triangles)
        gripper.vertex_colors = o3d.utility.Vector3dVector(colors)

        o3d_gripper_geom.append(gripper)

    #o3d.visualization.draw_geometries([pointcloud, *o3d_gripper_geom])

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

def create_plane(p1, p2, p3, p4):
    """ input:  p1  -   numpy array (x1, y1, z1)
                p2  -   numpy array (x2, y2, z2)
                p3  -   numpy array (x3, y3, z3)
                p4  -   numpy array (x4, y4, z4)
        output: box -   o3d mesh
    """ """not implemented yet !!!"""
    depth = 0.01 # in meters
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

    colors = [[1, 0, 0] for i in range(len(lines_to_viz))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points_to_viz)
    line_set.lines = o3d.utility.Vector2iVector(lines_to_viz)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set

def viz_oriented_boundingbox(corners):

    b1,b2,b4,t1,t3,t4,t2,b3 = corners
    points_to_viz = [b1, b2, b4, t1, t3, t4, t2, b3]

    lines_to_viz = [[0, 1], [0, 2], [1,7], [7, 2],
                    [3, 6], [3, 5], [6, 4], [5, 4],
                    [0, 3], [1, 6], [7, 4], [2, 5]]

    colors = [[1, 0, 0] for i in range(len(lines_to_viz))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points_to_viz)
    line_set.lines = o3d.utility.Vector2iVector(lines_to_viz)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set

def vizualizeLine(p1, p2):
    """ Input:  p1  -   numpy array (x, y, z)
                p2  -   numpy array (x, y, z)
        Output: lineset -   o3d.geometry.Lineset
    """
    points_to_viz = [p1, p2]

    lines_to_viz = [[0, 1]]

    colors = [[1, 0, 1] for i in range(len(lines_to_viz))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points_to_viz)
    line_set.lines = o3d.utility.Vector2iVector(lines_to_viz)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set

def visualizeFrame(x, y, z, translation, size = 0.1):
    """ Input:  x   -   numpy array (x, y, z) or column vector
                y   -   numpy array (x, y, z) or column vector
                z   -   numpy array (x, y, z) or column vector
                translation -   numpy array (x, y, z) or column vector
                                the center point of the frame
                size        -   float, length of lines in meters
        output: lines   -   open3d.geometry.LineSet()
    """

    translation = translation.flatten()

    x = (x.flatten() * size) + translation
    y = (y.flatten() * size) + translation
    z = (z.flatten() * size) + translation
    points = [translation, x, y, z]
    print(points)
    lines_to_viz = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0 , 0], [0, 1, 0], [0, 0, 1]]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines_to_viz)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


# Make points ready to visualize in open3d pointcloud with color
def viz_color_pointcloud(points, color):
    colors = [color for i in range(len(points))]
    points = np.array(points)
    pc_viz = o3d.geometry.PointCloud()
    pc_viz.colors = o3d.utility.Vector3dVector(colors)
    pc_viz.points = o3d.utility.Vector3dVector(points.astype(np.float32))
    return pc_viz
