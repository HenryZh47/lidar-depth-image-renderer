# LiDAR Depth Image Renderer

The package projects one or more LiDAR point clouds in map frame to a camera,
generating a depth image for the camera. Parallel versions are implemented in
both OpenMP and CUDA.

This is a class project for 15-618 Parallel Computer Architecture and
Programming, Spring 2020.

Demo Video:

[![Demo Video](http://img.youtube.com/vi/pxZWXLIlUps/0.jpg)](http://www.youtube.com/watch?v=pxZWXLIlUps "Parallel LiDAR Depth Image Rendering Tool Demo")

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

### Benchmark
1. Run `./benchmark.sh -b <your_dataset.bag> -t <play_duration>` under `scripts`
1. Example: `./benchmark.sh -b ~/Downloads/objdet_2019-08-17-16-37-39_3.bag -t 10`
1. It should automatically build and run tests for all three implementations
1. Observe produced CSV logging files under current working directory
1. Process and visualize raw results by running `python3 visualize.py` under `scripts`

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

## Results
We achieved on average 120.0x speed up using CUDA implementation.
![speed-up-comparison](https://github.com/HenryZh47/lidar-depth-image-renderer/blob/master/scripts/results/impl_ftime_comparison.png)

## Authors

- Henry Zhang (hengruiz@andrew.cmu.edu)
- Haowen Shi (haowensh@andrew.cmu.edu)
