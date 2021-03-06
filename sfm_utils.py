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

# This function reconstructs a point XYZ based on:
# cameras: array of camera params [r[0], r[1], r[2], t[1], t[2], t[3], f, k1, k2]
# points: array of xy coords for each camera [x,y]
def reconstruct_DLT(num_cameras, cameras, points):
    M = []
    Y = []
    for i in range(num_cameras):
        # Grab the i-th camera
        cam = cameras[i,:]
        
        # Obtain the projection matrix
        K = np.array([-cam[6], 0, 0, 0, -cam[6], 0, 0,0,1]).reshape(3,3)
        R = cv2.Rodrigues(cam[0:3])[0]
        t = cam[3:6]
        T = np.array([R[0][0], R[0][1], R[0][2], t[0],
                      R[1][0], R[1][1], R[1][2], t[1],
                      R[2][0], R[2][1], R[2][2], t[2]]).reshape(3,4) 
        P = np.matmul(K,T)
        P = np.ravel(P) 

        # Get the x y pixel coords
        x, y = points[i][0], points[i][1]

        # Construct the matrices
        M.append([x*P[8]-P[0], x*P[9]-P[1], x*P[10]-P[2]])
        M.append([y*P[8]-P[4], y*P[9]-P[5], y*P[10]-P[6]])
        Y.append(P[3]-x*P[11])
        Y.append(P[7]-y*P[11])


    # Solve the linear problem (Y = M*X --> inv([M'M])*M'*Y)
    # (Multiply by the pseudo-inverse)
    M = np.array(M)
    Y = np.array(Y)
    MTM = np.matmul(np.transpose(M), M)
    X = np.matmul(np.matmul(np.linalg.inv(MTM),np.transpose(M)),Y)
    
    return X


# Reprojects the 3D point using each camera in cameras array
# camera params as [r[0], r[1], r[2], t[1], t[2], t[3], f, k1, k2]
def reproject_points(point, cameras):
    projections = []
    for cam in cameras:
        # Obtain the projection matrix
        K = np.array([1, 0, 0, 0, 1, 0, 0,0,1]).reshape(3,3)
        R = cv2.Rodrigues(cam[0:3])[0]
        t = cam[3:6]
        T = np.array([R[0][0], R[0][1], R[0][2], t[0],
                      R[1][0], R[1][1], R[1][2], t[1],
                      R[2][0], R[2][1], R[2][2], t[2]]).reshape(3,4) 
        P = np.matmul(K,T)

        # Project
        X = np.array([point[0], point[1], point[2], 1]).reshape(4,1)
        uvw = np.matmul(P,X)
        uv = -np.array([uvw[0], uvw[1]])/uvw[2]
        n = np.sum(uv)**2
        r = 1 + cam[7] * n + cam[8] * n**2
        uv = uv*r*cam[6]

        projections.append(np.array([uv[0][0],uv[1][0]]))
    return np.array(projections)

# This function has been adapted from 
# https://scipy-cookbook.readthedocs.io/items/bundle_adjustment.html
def rotate(points, rot_vecs):
    """Rotate points by given rotation vectors.
    
    Rodrigues' rotation formula is used.
    """
    theta = np.linalg.norm(rot_vecs, axis=1)[:, np.newaxis]
    with np.errstate(invalid='ignore'):
        v = rot_vecs / theta
        v = np.nan_to_num(v)
    dot = np.sum(points * v, axis=1)[:, np.newaxis]
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    return cos_theta * points + sin_theta * np.cross(v, points) + dot * (1 - cos_theta) * v

# This function has been adapted from 
# https://scipy-cookbook.readthedocs.io/items/bundle_adjustment.html
def project(points, camera_params):
    """Convert 3-D points to 2-D by projecting onto images."""
    points_proj = rotate(points, camera_params[:, :3])
    points_proj += camera_params[:, 3:6]
    points_proj = -points_proj[:, :2] / points_proj[:, 2, np.newaxis]
    f = camera_params[:, 6]
    k1 = camera_params[:, 7]
    k2 = camera_params[:, 8]
    n = np.sum(points_proj**2, axis=1)
    r = 1 + k1 * n + k2 * n**2
    points_proj *= (r * f)[:, np.newaxis]
    return points_proj

# This function has been adapted from 
# https://scipy-cookbook.readthedocs.io/items/bundle_adjustment.html
def fun(params, n_cameras, n_points, camera_indices, point_indices, points_2d):
    """Compute residuals.
    
    `params` contains camera parameters and 3-D coordinates.
    """
    camera_params = params[:n_cameras * 9].reshape((n_cameras, 9))
    points_3d = params[n_cameras * 9:].reshape((n_points, 3))
    points_proj = project(points_3d[point_indices], camera_params[camera_indices])
    return (points_proj - points_2d).ravel()
    
# This function has been adapted from 
# https://scipy-cookbook.readthedocs.io/items/bundle_adjustment.html
from scipy.sparse import lil_matrix
def bundle_adjustment_sparsity(n_cameras, n_points, camera_indices, point_indices):
    m = camera_indices.size * 2
    n = n_cameras * 9 + n_points * 3
    A = lil_matrix((m, n), dtype=int)

    i = np.arange(camera_indices.size)
    for s in range(9):
        A[2 * i, camera_indices * 9 + s] = 1
        A[2 * i + 1, camera_indices * 9 + s] = 1

    for s in range(3):
        A[2 * i, n_cameras * 9 + point_indices * 3 + s] = 1
        A[2 * i + 1, n_cameras * 9 + point_indices * 3 + s] = 1

    return A