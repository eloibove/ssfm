#!/usr/bin/env python
import rospy
import numpy as np
import os 
from sfm_utils import CameraInfo, PointData, triangulate_points

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

        with open(filename) as input_file:

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
                        self.cameras.append(CameraInfo(self.f, self.K, self.R, self.t))
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

                        self.points.append(pointdata)
                                            


    # This method computes the first X points listed in the dataset
    def compute_points(self, nPoints):
        point_data = []
        for i, point in enumerate(self.points):
            if i > nPoints : break

            # TODO: Compute x, y, z based on the xy image coords
            # First try: only based on two images
            caminfo1 = self.cameras[int(point.instances[0]["id"])]
            caminfo2 = self.cameras[int(point.instances[1]["id"])]
            
            xy1 = point.instances[0]["xy"]
            xy2 = point.instances[1]["xy"]

            point3d = triangulate_points(caminfo1, caminfo2, xy1, xy2)

            print(point3d)
            print([point.x, point.y, point.z])
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', point.b, point.g, point.r, a))[0] 
            pt = [point3d[0], point3d[1], point3d[2], rgb]
            point_data.append(pt)
            break
            

        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),]
        
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
    sfm.compute_points(30000)