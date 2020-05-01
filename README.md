# LiDAR Depth Image Renderer

The package projects one or more LiDAR point clouds to a camera, generating a
depth image for the camera. Parallel versions are implemented in both OpenMP and
CUDA.

This is a class project for 15-618 Parallel Computer Architecture and
Programming, Spring 2020.

## Getting Started
### Dependencies
1. Ubuntu 18.04
1. ROS Melodic
   Follow the installation guide [here](http://wiki.ros.org/melodic/Installation).
1. OpenCV 3
1. OpenMP

### Building
1. Install `python-catkin-tools`
1. Create a ROS workspace and clone our package into `src` directory
1. Build with `catkin build`

### Launching
We use ROS launch files to run executables.
1. For sequential version, use launch file `subt.launch`
1. For OpenMP version, use launch file `subt_omp.launch`

## CUDA Version

TODO

## OpenMP Version
CPU parallel version using OpenMP multi-threading directives.

## Authors

- Henry Zhang (hengruiz@andrew.cmu.edu)
- Haowen Shi (haowensh@andrew.cmu.edu)
