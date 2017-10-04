/*
 * KinfuServer.h
 *
 *  Created on: Jun 3, 2015
 *      Author: mklingen
 */

#ifndef KINFUSERVER_H_
#define KINFUSERVER_H_

#include <iostream>

#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros_rgbd_camera.hpp>

#include <kinfu_ros/GetTSDF.h>
#include <kinfu_ros/GetTSDFRequest.h>
#include <kinfu_ros/GetTSDFResponse.h>
#include <kinfu_ros/TSDF.h>
#include <kfusion/kinfu.hpp>
#include <kfusion/types.hpp>

namespace kfusion
{
class KinFuServer
{
 public:
  KinFuServer(RosRGBDCamera* camera, const std::string& fixedFrame,
              const std::string& camFrame, const std::string& kinfuOdomTopic);

  template <typename T>
  void LoadParam(T& value, const std::string& name)
  {
    T newValue = camera_->nodeHandle.param<T>(name, value);
    value      = newValue;
  }

  inline void LoadParam(float& value, const std::string& name)
  {
    double curr = value;
    curr        = camera_->nodeHandle.param<double>(name, curr);
    value       = static_cast<float>(curr);
    std::cout << name << ": " << value << std::endl;
  }

  // Should be called before running the server.
  inline void Initialize()
  {
    ConnectCamera();
    LoadParams();
  }

  // Does one full step of grabbing a camera image,
  // then tracking against the model, followed by
  // publishing relevant data to ROS.
  void Update();

  // Publish a raycasted image of the scene to ROS
  void PublishRaycastImage();
  // Connects to the camera, and runs until told to stop.
  bool ExecuteBlocking();
  // Connects to the ROS depth camera, blocking.
  bool ConnectCamera();
  // Creates the KinFu server from the ROS parameters
  bool LoadParams();
  // Publishes the current camera transform as well as odometry message
  bool PublishTransformOdometry();
  /* \brief Publish the pose of the camera with a ros Publisher
   * \param t : the ros::Time to stamp message
   */
  void publishOdom(cv::Affine3f::Vec3 const & translation,
                   tf::Quaternion const & rotation,
                   ros::Time t);
  // Does a single KinFu step given a depth and (optional) color image.
  bool KinFu(const cv::Mat& depth, const cv::Mat& color);

  inline bool ShouldExit() const { return should_exit_; }
  inline void SetExit(bool value) { should_exit_ = value; }
  inline void Exit() { should_exit_ = true; }
  inline KinFu::Ptr GetKinfu() { return kinfu_; }
  inline void SetKinfu(KinFu::Ptr value) { kinfu_ = value; }
  inline const RosRGBDCamera* GetCamera() { return camera_; }
  inline void SetCamera(RosRGBDCamera* value) { camera_ = value; }

  inline const cv::Mat& GetViewHost() { return viewHost_; }
  inline const cuda::Image& GetViewDevice() { return viewDevice_; }
  inline const cuda::Depth& GetDepthDevice() { return depthDevice_; }
  inline const std::string& GetBaseFrame() { return baseFrame_; }
  inline const std::string& GetCameraFrame() { return cameraFrame_; }
  // Service calls
  bool GetTSDF(kinfu_ros::GetTSDFRequest& req, kinfu_ros::GetTSDFResponse& res);

 protected:
  SampledTime grab_timer_;
  SampledTime kinfu_timer_;
  bool should_exit_;
  KinFu::Ptr kinfu_;
  RosRGBDCamera* camera_;
  cv::Mat viewHost_;
  cuda::Image viewDevice_;
  cuda::Depth depthDevice_;

  ros::Publisher raycastImgPublisher_;
  ros::Publisher odometryPublisher_;
  std::string baseFrame_;
  std::string cameraFrame_;

  tf::TransformBroadcaster tfBroadcaster_;
  cv::Mat lastDepth_;
  cv::Mat lastColor_;
  ros::ServiceServer get_tsdf_server_;
};

} /* namespace kfusion */
#endif /* KINFUSERVER_H_ */
