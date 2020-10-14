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

    # Get the parameters from launchfile
    num_points = int(rospy.get_param('~num_points', '127430'))
    filename = rospy.get_param('~filename', 'src/sfm/NotreDame/notredame.out')
    topic = rospy.get_param('~topic', '/reconstructed_cloud')

    # Init publisher
    pub = rospy.Publisher(topic, PointCloud2, queue_size=1)

    # Create the SfM object
    print('Reading model data...')
    sfm = SfM(filename)
    print('Model loaded!')

    # Compute the first num_points    
    print('Starting computation...')
    pc, error, np = sfm.compute_points(num_points)

    print('Reconstructed ' + str(np) + ' points!')
    print('Reconstruction MSE = ' + str(error))
    print('Subscribe to ' + topic + ' to see the results in RViz!')

    # Publish the pc in a loop to avoid the program to finish
    while not rospy.is_shutdown():
        pc.header.stamp = rospy.Time.now()
        pub.publish(pc)
        rospy.sleep(1.0)