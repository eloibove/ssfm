#!/usr/bin/env python
import rospy
import numpy as np
import os
from sfm_utils import CameraInfo, PointData, reconstruct_DLT
from SfM import SfM

import struct
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

if __name__ == '__main__':
    # Init ros
    rospy.init_node('sfm')

    # Get the parameters from launchfile)
    mode = rospy.get_param('~mode', 1)
    num_points = rospy.get_param('~num_points', 5000)
    filename = rospy.get_param('~filename', 'src/ssfm/NotreDame/notredame.out')
    topic = rospy.get_param('~topic', '/reconstructed_cloud')

    # Init publisher
    pubDLT = rospy.Publisher(topic+'/DLT', PointCloud2, queue_size=1)
    pubBA = rospy.Publisher(topic+'/BA', PointCloud2, queue_size=1)

    # Create the SfM object
    print('Reading model data...')
    sfm = SfM(filename, num_points)
    print('Model loaded!')

    # Compute the first num_points using the DLT   
    print('Starting DLT computation...')
    pc, np = sfm.compute_points(127431)
    print('Reconstructed ' + str(np) + ' points!')

    pc2 = None
    if mode == 1:

        print('Computing initial reprojection error...')
        rpe = sfm.compute_reprojection_err()
        print('Mean absolute reprojection error1 = ' + str(rpe) + ' pixels!')

        print('Starting bundle adjustment...')
        sfm.bundle_adjustment()

        print('Computing final reprojection error...')
        rpe = sfm.compute_reprojection_err()
        print('Mean absolute reprojection error = ' + str(rpe) + ' pixels!')

        pc2 = sfm.get_pointcloud()

    print('Subscribe to ' + topic + ' to see the results in RViz!')
    # Publish the pc in a loop to avoid the program to finish
    while not rospy.is_shutdown():
        pc.header.stamp = rospy.Time.now()
        pubDLT.publish(pc)
        if mode == 1:
            pubBA.publish(pc2)
        rospy.sleep(1.0)