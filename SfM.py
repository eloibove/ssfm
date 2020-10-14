#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import os 
from sfm_utils import CameraInfo, PointData, reconstruct_DLT, reproject_points
from sfm_utils import bundle_adjustment_sparsity, fun

import struct
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from scipy.optimize import least_squares


## This class is used to read the notredame image dataset and compute its 3D reconstruction ##
## The reconstruction is returned as a PointCloud2 ROS message ##
class SfM(object):

    # The init method reads the camera parameters from the provided filename
    # The input file format should be as in the bundler-v0.3:
    # http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.3-manual.html#S4
    def __init__(self, filename, num_p):
        self.filename = filename
        self.cameras = []
        self.points = []
        self.points3d = []
        self.color3d = []
        self.points2d = []
        self.camera_indices = []
        self.point_indices = []

        with open(filename) as input_file:
            point_id = 0
            for i, line in enumerate(input_file):
                if i == 0 : continue
                if i == 1 :
                    line = line.strip().split()
                    self.num_cameras = float(line[0])
                    self.num_points = float(line[1])
                    continue

                # First entries, read camera data
                # After num_cameras, read points
                if i < self.num_cameras*5+2 :
                    if (i-2)%5 == 0 : 
                        line = line.strip().split()
                        self.f = float(line[0])
                        self.K = np.array([float(line[1]), float(line[2])])

                    elif (i-2)%5 == 1 : 
                        self.R1 = np.fromstring(line, dtype=float, sep=' ')

                    elif (i-2)%5 == 2 : 
                        self.R2 = np.fromstring(line, dtype=float, sep=' ')

                    elif (i-2)%5 == 3 : 
                        self.R3 = np.fromstring(line, dtype=float, sep=' ')
                        
                    elif (i-2)%5 == 4 :
                        self.t = np.fromstring(line, dtype=float, sep=' ')
                        self.R = np.array([self.R1, self.R2, self.R3])
                        #self.cameras.append(CameraInfo(self.f, self.K, self.R, self.t))
                        rvec = cv2.Rodrigues(self.R)
                        self.cameras.append(np.array([float(rvec[0][0]), float(rvec[0][1]), float(rvec[0][2]),
                                             self.t[0], self.t[1], self.t[2],
                                             self.f, self.K[0], self.K[1]]))
                        
                else:
                    # Read the points
                    if (i-1)%3 == 0 : 
                        self.XYZ = np.fromstring(line, dtype=float, sep=' ')

                    elif (i-1)%3 == 1 : 
                        self.RGB = np.fromstring(line, dtype=int, sep=' ')

                    elif (i-1)%3 == 2 :
                        pointdata = PointData(self.XYZ[0], self.XYZ[1],
                                              self.XYZ[2], self.RGB[0],
                                              self.RGB[1], self.RGB[2])
                        line = line.strip().split()
                        num_points = int(line[0])
                        for j in range(1, num_points*4-1, 4):
                            cam_id = int(line[j])
                            x = float(line[j+2])
                            y = float(line[j+3])
                            pointdata.add_instance(cam_id, x, y)
                            self.camera_indices.append(cam_id)
                            self.points2d.append(np.array([x,y]))
                            self.point_indices.append(point_id)
                        point_id = point_id + 1

                        self.points.append(pointdata)
                        self.points3d.append(self.XYZ)
                        self.color3d.append(self.RGB)
                        if(point_id >= num_p):
                            break

            self.points3d = np.array(self.points3d)
            self.points2d = np.array(self.points2d)
            self.color3d = np.array(self.color3d)
            self.camera_indices = np.array(self.camera_indices)
            self.point_indices = np.array(self.point_indices)
            self.cameras = np.array(self.cameras)
            self.est_points3d = []
                                            

    # This method computes the DLT reconstruction
    # retuns the reconstruction mean squared error
    def compute_points(self):

        point_data = []
        # Iterate on all the points
        for i,point in enumerate(self.points):

            # Stop at nPoints
            total_points = i

            num_instances = np.shape(point.instances)[0]
            point_cameras = []
            points_to_compute = []
            camera_indices = []

            # Iterate on all the instances of each point
            for inst in point.instances:
                point_cameras.append(self.cameras[int(inst["id"])])
                camera_indices.append(int(inst["id"]))
                points_to_compute.append(inst["xy"])
            point_cameras = np.array(point_cameras)
            points_to_compute = np.array(points_to_compute)
        
            # Compute the DLT for this point
            point3d = reconstruct_DLT(num_instances, point_cameras, points_to_compute)
            self.est_points3d.append(point3d)

            # Create the point cloud data
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', self.color3d[i][2], self.color3d[i][1], self.color3d[i][0], a))[0] 
            pt = [point3d[0], point3d[1], point3d[2], rgb]
            point_data.append(pt)
        
            
        # Define the point cloud metadata
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),]
        
        # Create the message and return
        header = Header()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, point_data)
        self.est_points3d = np.array(self.est_points3d)

        return pc2, total_points


    # Returns the mean absolute reprojection error for the predictions
    def compute_reprojection_err(self):

        cumulative_err = 0
        for i, point in enumerate(self.points):
            uvs_gt = []
            cameras = []
            for inst in point.instances:
                cameras.append(self.cameras[int(inst["id"])])
                uvs_gt.append(inst["xy"])
            cameras = np.array(cameras)
            uvs_gt = np.array(uvs_gt)
            uvs = reproject_points(self.est_points3d[i], cameras)
            point_err = np.sum(np.abs(uvs_gt-uvs))
            cumulative_err += point_err
        cumulative_err /= np.shape(self.est_points3d)[0]
        
        return cumulative_err

    # Return estimated points 3d as a ROS pointcloud message
    def get_pointcloud(self):

        point_data = []
        for i, point3d in enumerate(self.est_points3d):
            # Create the point cloud data
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', self.color3d[i][2], self.color3d[i][1], self.color3d[i][0], a))[0] 
            pt = [point3d[0], point3d[1], point3d[2], rgb]
            point_data.append(pt)
        
        # Define the point cloud metadata
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),]
        
        # Create the message and return
        header = Header()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, point_data)
        self.est_points3d = np.array(self.est_points3d)

        return pc2

    
    # Refine the parameters obtained by the DLT
    # This function has been adapted from 
    # https://scipy-cookbook.readthedocs.io/items/bundle_adjustment.html

    def bundle_adjustment(self):
        num_cameras = np.shape(self.cameras)[0]
        num_points = np.shape(self.est_points3d)[0]
        x0 = np.hstack((self.cameras.ravel(), self.est_points3d.ravel()))
        f0 = fun(x0, num_cameras, num_points, self.camera_indices, self.point_indices, self.points2d)
        print(np.abs(np.sum(f0))) 
        A = bundle_adjustment_sparsity(num_cameras, num_points, self.camera_indices, self.point_indices)
        res = least_squares(fun, x0, jac_sparsity=A, verbose=2, x_scale='jac', ftol=1e-4, method='trf',
                            args=(num_cameras, num_points, self.camera_indices, self.point_indices, self.points2d))
        print(np.abs(np.sum(res.fun)))
        self.est_points3d = res.x[num_cameras * 9:].reshape((num_points, 3))
