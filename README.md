# LiDAR Depth Image Renderer

The package projects one or more LiDAR point clouds in map frame to a camera,
generating a depth image for the camera. Parallel versions are implemented in
both OpenMP and CUDA.

This is a class project for 15-618 Parallel Computer Architecture and
Programming, Spring 2020.

## Getting Started

### Dependencies

1. Ubuntu 18.04
1. ROS Melodic
   Follow the installation guide [here](http://wiki.ros.org/melodic/Installation).
1. OpenCV 3
1. OpenMP
1. CUDA 10

### Building

1. Install `python-catkin-tools`
1. Create a ROS workspace and clone our package into `src` directory
1. Build with `catkin build`

### Launching

We use ROS launch files to configure parameters and run executables.

1. For sequential version, use launch file `subt.launch`
1. For OpenMP version, use launch file `subt_omp.launch`
1. For CUDA version, use launch file `subt_cuda.launch`

## CUDA Version

GPU parallel version with CUDA.

- To compile, checkout your GPU compute capability [here](https://developer.nvidia.com/cuda-gpus#compute),
  and add it to NVCC compile flags in CMakeList.

## OpenMP Version

CPU parallel version using OpenMP multi-threading directives.

## Authors

- Henry Zhang (hengruiz@andrew.cmu.edu)
- Haowen Shi (haowensh@andrew.cmu.edu)
