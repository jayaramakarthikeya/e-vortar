


#ifndef KITTI_STREAMER_STREAMER_NODE_H_
#define KITTI_STREAMER_STREAMER_NODE_H_

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "kitti_utils.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <chrono>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

using namespace cv;
using namespace std::chrono_literals;

class StreamerNode : public rclcpp::Node {

   public:
    enum class PublisherType 
    { 
        POINT_CLOUD = 0,
        IMAGE_LEFT_GRAY = 1, 
        IMAGE_RIGHT_GRAY = 2,  
        ODOMETRY = 3
    }; 

    KittiUtils kitti_utils;

    StreamerNode();

    private:

        size_t file_index_;

        void on_timer_callback();

        void convert_bin_to_pointcloud2();
        std::string mat_type2encoding(int mat_type);
        void convert_image_to_msg(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr,std::string);
        void publish_odom_msg();
        void publish_path_msg(geometry_msgs::msg::PoseStamped msg);
        
        void handle_velo_cam_transform();
        void handle_map_to_odom_transform();
        void process_calib();
        void handle_static_transform();
        void publish_camera_info_msg(rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr,std::string);

        std::vector<Eigen::Matrix4f> poses;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_;   // velodyne point clouds publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_left_;     // left rectified grayscale image sequence
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_right_;    // right rectified grayscale image sequence
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_image_gray_left_info; 
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_image_gray_right_info; 
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_odometry_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

        std::unordered_map<std::string, std::vector<double>> calib_data;
        nav_msgs::msg::Path path_msg;

        int image_height, image_width;
        rclcpp::Time cam_time;

        Eigen::Matrix4f camera_base_rotation;
        Eigen::MatrixXf T_cam1_velo;
        Eigen::Matrix3f K_cam0, K_cam1;
        Eigen::Matrix<float,3,4> Tr;
        tf2::Transform map_T_odom_last,cam0_T_velo,base_T_cam0;
        geometry_msgs::msg::TransformStamped map_stamped_T_odom,camera_lidar_transform;

        //Eigen::MatrixXf Tr(3,4);
};


#endif