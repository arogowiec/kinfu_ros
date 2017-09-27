KinFu ros
=========
This is a ROS-generic version of  [kinfu_remake](https://github.com/Nerei/kinfu_remake). It aims to make the package usable without any reference to `OpenNI`, with no assumptions about the kind of depth sensor used. It also aims to make data from the `kinfu` pipeline available via ROS service calls and messages.

This also means the project will probably no longer work in Windows.

The project has been tested with two ROS cameras so far. It seems to work with a [structure sensor](http://structure.io/developers), for instance. It should work with any sensor that provides a dense depth map with unsigned 16 bit millimeter depth images (an image of the structure io sensor being used to run `kinfu` is shown below). With `depth_image_proc` it might even be possible for it to work with stereo cameras.

![](https://raw.githubusercontent.com/personalrobotics/kinfu_ros/master/fusion_structureio.png)

## Running Standalone Kinfu Server

An example launch file is shown in `/launch/default.launch`. To launch the `kinfu_ros` server simply use the command

`roslaunch kinfu_ros <your_launch_file>`

This will spin up a `kinfu_ros` server, that will continuously receive images and track the pose of the camera. Right now, `kinfu_ros` only publishes two things to ROS: a raycasted image from `kinfu`, and a pose through `tf`. The launch file allows you to select topics for the depth image and color image, and you can modify many other parameters through `rosparam`.

You can also include the `kinfu_ros` server as a header file, and use it in any way you wish. Just include `<kinfu_ros/kinfu_server.h>` in your code. `src/kinfu_node.cpp` shows how to use the `kinfu_ros` server to receive and track depth images.

## Getting the TSDF through a service call

The `kinfu_ros` server advertises a service that lets you read the raw TSDF data through ros. The `kinfu/get_tsdf` service will return a message that has a giant `uint32` array. The first 16 bits are a half-precision float representing the signed distance field. The second 16 bits are an unsigned weight.

The `kinfu/get_tsdf` service is very slow, so use it sparingly.

KinFu remake
============

This is lightweight, reworked and optimized version of Kinfu that was originally shared in PCL in 2011. 

Key changes/features:
* Performance has been improved by 1.6x factor (Fermi-tested)
* Code size is reduced drastically. Readability improved. 
* No hardcoded algorithm parameters! All of them can be changed at runtime (volume size, etc.)
* The code is made independent from OpenCV GPU module and PCL library. 

Dependencies:
* Fermi or Kepler or newer
* CUDA 5.0 or higher
* OpenCV 2.4.9 ~~with new Viz module~~ (only opencv_core, ~~opencv_highgui~~, opencv_imgproc, ~~opencv_viz~~ modules required). ~~Make sure that WITH_VTK flag is enabled in CMake during OpenCV configuration.~~
* ~~OpenNI v1.5.4 (for Windows can download and install from http://pointclouds.org/downloads/windows.html)~~

~~Implicit dependency (needed by opencv_viz):~~
~~* VTK 5.8.0 or higher. (apt-get install on linux, for windows please download and compile from www.vtk.org)~~

Screenshot:
![](https://raw.githubusercontent.com/personalrobotics/kinfu_ros/master/perf-39.5fps-Tesla-C2070.png)

