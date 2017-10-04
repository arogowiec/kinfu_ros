#include <ros/kinfu_server.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <kfusion/kinfu.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros_rgbd_camera.hpp>

using namespace kfusion;

int main(int argc, char* argv[])
{
  int device = 0;
  cuda::setDevice(device);
  cuda::printShortCudaDeviceInfo(device);

  if (cuda::checkIfPreFermiGPU(device))
    return std::cout << std::endl
                     << "Kinfu is not supported for pre-Fermi GPU "
                        "architectures, and not built for them by default. "
                        "Exiting..."
                     << std::endl,
           1;

  ros::init(argc, argv, "kinfu_ros");

  ros::NodeHandle node("~");
  RosRGBDCamera camera(node);
  camera.SubscribeDepth("/camera/depth/image_raw");
  camera.SubscribeRGB("/camera/rgb/image_rect_color");
  
  std::string fixedFrame  = "/map";
  std::string cameraFrame = "/camera_depth_optical_frame";
  std::string kinfuOdomTopic = "kinfu/odom";

  node.param<std::string>("fixed_frame", fixedFrame, "/map");
  node.param<std::string>("camera_frame", cameraFrame,
                          "/camera_depth_optical_frame");
  node.param<std::string>("kinfu_odom", kinfuOdomTopic, "kinfu/odom");
  KinFuServer app(&camera, fixedFrame, cameraFrame, kinfuOdomTopic);
  app.ExecuteBlocking();

  return 0;
}
