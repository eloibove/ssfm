# ssfm

This ros package implements an SfM class that allows the user to get the reconstructed 3D points given a set of images, point correspondences, and camera parameters.

## Installation

Clone the ros package into a catkin workspace and compile it. 

```bash
git clone https://github.com/eloibove/ssfm
```

Aside from basic ros packages, this package does not need any dependencies.

## Usage

First run the roscore
```bash
roscore
```
Then run the main node with the default parameters, and run rviz to visualize the result (a colored PointCloud2), subscribing to /reconstructed_cloud

```bash
rosrun sfm main.py
```
```bash
rviz
```
### Parameters
The input filename and point cloud topic name are ROS parameters, and can be specified when running the node in the following way:
```bash
 rosrun sfm main.py _filename:=file.txt _topic:=/topic
```

## How it works
This package relies on the Direct Linear Transform method. This method allows the triangulation of a 3D point based on 2D correspondences of the same point on various cameras (different viewpoints). This is done by arranging the problem x1=P1*X, x2=P2*X in a particular way so that it can be solved using Least Squares. This method is non-iterative and gives decent results with a low computation time. If the accuracy needs to be higher, a bundle adjustment algorithm that takes these results as input can improve the accuracy, also taking into account the lens distortion parameters, that the DLT method ignores.

## References
http://kwon3d.com/theory/dlt/dlt.html
