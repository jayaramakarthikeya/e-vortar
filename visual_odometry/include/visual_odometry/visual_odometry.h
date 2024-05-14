#pragma once

#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/rotation.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include "ceres_cost_function.h"
#include "visual_odometry/image_util.h"
#include "visual_odometry/point_cloud_util.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tic_toc.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "kitti_streamer/streamer_node.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
 #include <message_filters/time_synchronizer.h>

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
// sensor_msgs::PointCloud2> VO_policy;

class VisualOdometry : public rclcpp::Node
{
public:
  VisualOdometry();

  void reset();
  void process_static_transform();

  void processImage(const cv::Mat& img00);

  // void setUpPointCloud(const Eigen::Isometry3f& imu_eigen_T_cam0, const Eigen::Isometry3f& imu_eigen_T_velo, const
  // sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  void setUpPointCloud(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);

  void processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg,
                         const pcl::PointCloud<pcl::PointXYZ>& point_cloud_pcl, const bool& visualize_depth,
                         const bool& publish_point_cloud);

  void solveRANSAC();
  void solveNlsAll();
  void solveNls2dOnly();
  

  void publish();
  void VO2Cam0StartFrame();
  void VO2VeloAndBase(const tf2::Transform &cam0_curr_VOT_cam0_last);

  // private:
  //std::shared_ptr<VloamTF> vloam_tf;

  int i, j, count;

  ImageUtil image_util;
  std::vector<cv::Mat> images;
  std::vector<std::vector<cv::KeyPoint>> keypoints;
  std::vector<cv::Mat> descriptors;
  std::vector<cv::DMatch> matches;
  std::vector<std::vector<cv::Point2f>> keypoints_2f;
  std::vector<uchar> optical_flow_status;

  std::vector<PointCloudUtil> point_cloud_utils;
  Eigen::MatrixXf point_cloud_3d_tilde;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  float depth0, depth1;
  double angles_0to1[3];
  double t_0to1[3];
  Eigen::Isometry3f velo_eigen_T_cam0;
  tf2::Transform cam0_T_base;
  Eigen::Vector3f point_3d_image0_0;
  Eigen::Vector3f point_3d_image0_1;
  Eigen::Vector3f point_3d_rect0_0;
  Eigen::Vector3f point_3d_rect0_1;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pub_point_cloud;

  // ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.5);
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  float angle;
  tf2::Transform cam0_curr_T_cam0_last;
  tf2::Quaternion cam0_curr_q_cam0_last;
  tf2::Transform world_VOT_odom_last;
  geometry_msgs::msg::TransformStamped world_stamped_VOtf_odom;
  tf2::Transform odom_T_base,velo_T_cam0;
  tf2::Transform cam0_init_VOT_cam0_last, cam0_init_VOT_cam0_start, cam0_start_VOT_cam0_last,velo_last_VOT_velo_curr,base_last_VOT_base_curr;
  Eigen::Isometry3f cam0_start_eigen_VOT_cam0_last;
  

private:

  int verbose_level;
  bool reset_VO_to_identity;
  int remove_VO_outlier;
  bool keypoint_NMS;
  bool CLAHE;
  cv::Ptr<cv::CLAHE> clahe;
  bool visualize_optical_flow;
  bool optical_flow_match;


  void visual_timer_callback(); 
  void vi_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
              const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg);
  nav_msgs::msg::Odometry visualOdometry;
  nav_msgs::msg::Path visualPath;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubvisualOdometry;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubvisualPath;
  image_transport::Publisher pub_matches_viz;
  cv_bridge::CvImage matches_viz_cvbridge;
  image_transport::Publisher pub_depth_viz;
  cv_bridge::CvImage depth_viz_cvbridge;
  image_transport::Publisher pub_optical_flow_viz;
  cv_bridge::CvImage optical_flow_viz_cvbridge;

  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped tf_stamped_lidar_T_cam0,tf_stamped_cam0_T_base,tf_stamped_base_T_odom;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
                                                        sensor_msgs::msg::PointCloud2>
    MySyncPolicy;
  //message_filters::Synchronizer<MySyncPolicy> visual_sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_image00_ptr;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> sub_camera00_ptr;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_lidar_ptr;
  cv_bridge::CvImagePtr cv_ptr;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync;
};

#endif