#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import os 
from sfm_utils import CameraInfo, PointData, reconstruct_DLT

import struct
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


## This class is used to read the notredame image dataset and compute its 3D reconstruction ##
## The reconstruction is published as a PointCloud2 ROS message ##
class SfM(object):

    # The init method reads the camera parameters from the provided filename
    def __init__(self, filename):
        self.filename = filename
        self.pub = rospy.Publisher('/reconstructed_cloud', PointCloud2, queue_size=1)
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

            self.points3d = np.array(self.points3d)
            self.points2d = np.array(self.points2d)
            self.color3d = np.array(self.color3d)
            self.camera_indices = np.array(self.camera_indices)
            self.point_indices = np.array(self.point_indices)
            self.cameras = np.array(self.cameras)
                                            


    # This method computes the first X points listed in the dataset
    def compute_points(self, nPoints):
        point_data = []
        # Iterate on all the points
        for i,point in enumerate(self.points):

            num_instances = np.shape(point.instances)[0]
            point_cameras = []
            points_to_compute = []

            # Iterate on all the instances of each point
            for inst in point.instances:
                point_cameras.append(self.cameras[int(inst["id"])])
                points_to_compute.append(inst["xy"])
            point_cameras = np.array(point_cameras)
            points_to_compute = np.array(points_to_compute)

            # Compute the DLT for this point
            point3d = reconstruct_DLT(num_instances, point_cameras, points_to_compute)

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
        
        # Create and publish the message in a loop
        header = Header()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, point_data)
        print('Reconstructed!')
        while not rospy.is_shutdown():
            pc2.header.stamp = rospy.Time.now()
            self.pub.publish(pc2)
            rospy.sleep(1.0)
        





if __name__ == '__main__':
    # Init ros
    rospy.init_node('sfm')

    # Create the SfM object
    # TODO: filename as parameter in launch file
    sfm = SfM('./NotreDame/notredame.out') 

    # Compute the first 100 points
    sfm.compute_points(1000)