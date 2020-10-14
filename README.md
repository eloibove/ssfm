# ssfm

This ros package implements an SfM class that allows the user to get the reconstructed 3D points given a set of images, point correspondences, and camera parameters. This package takes as input files the .out files created by bundler. The example dataset can be found at http://phototour.cs.washington.edu/datasets/

## Installation

Clone the ros package into a catkin workspace and compile it. 

```bash
git clone https://github.com/eloibove/ssfm
```

Aside from basic ros packages, this package does not have any dependencies.

## Usage

First run the roscore
```bash
roscore
```
Then run the main node with the default parameters, and run rviz to visualize the result (a colored PointCloud2), subscribing to /reconstructed_cloud/DLT

```bash
rosrun sfm main.py
```
```bash
rviz
```
### Parameters
##### Filename
The input filename can be changed by adding _filename:=file.out to the rosrun command. This is used to change the path of the dataset or the dataset itself.
##### Mode
The functioning mode changes as follows: _mode:=0 computes the reconstruction using only the DLT. _mode:=1 computes the first iteration of the reconstruction using the DLT, but then uses bundle adjustment to refine the 3D coordinates. The number of points should be set accordingly to avoid system crushes.
##### Number of points
This is the number of points to use in the computation. When using mode=0, this can be set to the maximum (around 100K points) without problem. When using mode=1, this should be set to around 5K points to have a feasible computation time. This is set by _num_points:=5000.
##### Topic
The point cloud topic name can be changed by _topic:=/topic
##### TL:DR
Compute DLT only with 100k points:
```bash
 rosrun sfm main.py _mode:=0 _num_points:=100000
```
Compute DLT and then refine with 5000 points:
```bash
 rosrun sfm main.py _mode:=1 _num_points:=5000
```

## How it works
This package relies on the Direct Linear Transform method. This method allows the triangulation of a 3D point based on 2D correspondences of the same point on various cameras (different viewpoints). This is done by arranging the problem x1=P1*X, x2=P2*X in a particular way so that it can be solved using Least Squares. This method is non-iterative and gives decent results with a low computation time. If the accuracy needs to be higher, a bundle adjustment algorithm that takes these results as input can improve the accuracy, also taking into account the lens distortion parameters, that the DLT basic method ignores.


## References
http://kwon3d.com/theory/dlt/dlt.html
https://scipy-cookbook.readthedocs.io/items/bundle_adjustment.html

## Images
![DLT only](https://github.com/eloibove/ssfm/blob/main/images/DLT.png?raw=true)
![DLT + Bundle adjustment 1](https://github.com/eloibove/ssfm/blob/main/images/DLT%2BBA.png?raw=true)
![DLT + Bundle adjustment 2](https://github.com/eloibove/ssfm/blob/main/images/DLT%2BBA2.png?raw=true)
