#!/usr/bin/env python
import numpy as np
import cv2
import glob

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

def reconstruct_DLT(num_cameras, cameras, points):
    M = []
    Y = []
    for i in range(num_cameras):
        # Grab the i-th camera
        cam = cameras[i,:]
        
        # Obtain the projection matrix
        K = np.array([cam[6], 0, 0, 0, cam[6], 0, 0,0,1]).reshape(3,3)
        R = cv2.Rodrigues(cam[0:3])[0]
        t = cam[3:6]
        T = np.array([R[0][0], R[0][1], R[0][2], t[0],
                      R[1][0], R[1][1], R[1][2], t[1],
                      R[2][0], R[2][1], R[2][2], t[2]]).reshape(3,4) 
        P = np.matmul(K,T)
        P = np.ravel(P) 
        """
        distCoeffs2 = np.array([cam[7],cam[8],0,0,0])
        distCoeffs1 = np.array([0,0,0,0,0])
        
        # agafar la mida dimatge de la foto jeje 

        r1 = cv2.stereoRectify(np.eye(3), distCoeffs1, K, distCoeffs2, (960,1280), R, t)
        
        P=np.matmul(r1[1],r1[3]).ravel()
        """
        # Get the x y pixel coords
        x, y = points[i][0], points[i][1]

        # Construct the matrices
        # M.append( [P[0]-x*P[8], P[1]-x*P[9], P[2]-x*P[10], P[3]-x*P[11]] )
        # M.append( [P[4]-y*P[8], P[5]-y*P[9], P[6]-y*P[10], P[7]-y*P[11]] )
        
        M.append([x*P[8]-P[0], x*P[9]-P[1], x*P[10]-P[2]])
        M.append([y*P[8]-P[4], y*P[9]-P[5], y*P[10]-P[6]])
        Y.append(P[3]-x*P[11])
        Y.append(P[7]-y*P[11])

        """
        M.append( [P[0]-x*P[8], P[1]-x*P[9], P[2]-x*P[10], P[3]-x] )
        M.append( [P[4]-y*P[8], P[5]-y*P[9], P[6]-y*P[10], P[7]-y] )
        """

    # Solve the linear problem (Y = M*X --> inv([M'M])*M'*Y)
    # (Multiply by the pseudo-inverse)
    M = np.array(M)
    Y = np.array(Y)
    MTM = np.matmul(np.transpose(M), M)
    X = np.matmul(np.matmul(np.linalg.inv(MTM),np.transpose(M)),Y)
    
    return X

    """
    #Find the xyz coordinates:
    U, S, Vh = np.linalg.svd(np.asarray(M))

    #Point coordinates in space:
    xyz = Vh[-1,0:-1] / Vh[-1,-1]
    print(Vh)
    print(xyz)
    return xyz
    """