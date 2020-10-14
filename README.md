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
Then run the main node with the default parameters, and run rviz to visualize the result, subscribing to /reconstructed_cloud

```bash
rosrun sfm main.py
```
```bash
rviz
```
### Parameters
The input filename, number of points to compute and point cloud topic name are ROS parameters, and can be specified when running the node in the following way:
```bash
 rosrun sfm main.py _filename:=file.txt _num_points:=100 _topic:=/topic
```
