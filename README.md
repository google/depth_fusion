## DepthFusion

This is not an official Google product.

## Overview

**DepthFusion** is an open source software library for reconstructing 3D surfaces (meshes) from depth data produced by commercial off-the-shelf depth cameras such as Microsoft Kinect, Asus Xtion Pro, and Intel RealSense.

It is at its core an implementation of "A Volumetric Method for Building Complex Models from Range Images" by Curless and Levoy [SIGGRAPH 1996]. Camera pose can be determined using the frame-to-model technique popularized by KinectFusion [Newcombe et al. 2011], or using fiducial markers from OpenCV's ArUco contrib module.

It runs on real-time on high-end desktop GPUs.

## Code structure

C++ and CUDA code are in **src**.
GLSL shaders for the visualization are in **src/shaders**.

## Dependencies

* Qt 5.5
* CUDA 7.5
* libcgt (see below) and its transitive dependencies.
* GLEW
* OpenCV 3.0 

## Build instructions (CMake)
0. Get [https://github.com/jiawen/libcgt](https://github.com/jiawen/libcgt "libcgt").
1. Put libcgt the same level as depth_fusion (they should be siblings). siblings).
2. mkdir build
3. cd build
4. cmake-gui ..

## License

Apache 2.0.
