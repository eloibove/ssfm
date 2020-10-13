#!/usr/bin/env python
import numpy as np
import cv2

## This class stores the camera parameters ##
class CameraInfo(object):
    def __init__(self, f, K, R, t):
        self.f = f
        self.K = K
        self.R = R
        self.t = t

## This class stores the points ##
class PointData(object):
    def __init__(self, x, y, z, r, g, b):
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.g = g
        self.b = b
        self.instances = []

    def add_instance(self, cam_id, x, y):
        self.instances.append({"id": cam_id, "xy": np.array([x,y])})

def triangulate_points(camera1, camera2, xy1, xy2):
    K1 = np.array([camera1.f, 0, 0,0, camera1.f, 0, 0,0,1]).reshape(3,3)
    T1 = np.array([camera1.R[0][0], camera1.R[0][1], camera1.R[0][2], camera1.t[0],
                  camera1.R[1][0], camera1.R[1][1], camera1.R[1][2], camera1.t[1],
                  camera1.R[2][0], camera1.R[2][1], camera1.R[2][2], camera1.t[2]]).reshape(3,4)
    K2 = np.array([camera2.f, 0, 0,0, camera2.f, 0, 0,0,1]).reshape(3,3)
    T2 = np.array([camera2.R[0][0], camera2.R[0][1], camera2.R[0][2], camera2.t[0],
                  camera2.R[1][0], camera2.R[1][1], camera2.R[1][2], camera2.t[1],
                  camera2.R[2][0], camera2.R[2][1], camera2.R[2][2], camera2.t[2]]).reshape(3,4)

    P1 = np.matmul(K1,T1)
    P2 = np.matmul(K2,T2)
    
    print(P1)
    print(xy1)
    print(P2)
    print(xy2)

    points4d = cv2.triangulatePoints(P1, P2, xy1, xy2)
    points3d = cv2.convertPointsFromHomogeneous(np.transpose(points4d))
    return points3d[0][0]