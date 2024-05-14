
#include "../include/kitti_streamer/streamer_node.h"

using namespace cv;

StreamerNode::StreamerNode() : Node("streamer_node"), file_index_(0) {
    publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/top/point_cloud_raw", 10);
    publisher_image_gray_left_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", 10);
    publisher_image_gray_right_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", 10);
    publisher_odometry_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("car/base/odom",10);
    publisher_path_ = this->create_publisher<nav_msgs::msg::Path>("car/base/odom_path",10);
    publisher_image_gray_left_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/left/camera_info",10);
    publisher_image_gray_right_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/right/camera_info",10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    this->declare_parameter("sequence","00");
    this->declare_parameter("data_dir","");
    this->declare_parameter("odom_dir","");

    std::string data_dir = this->get_parameter("data_dir").as_string();
    std::string sequence = this->get_parameter("sequence").as_string();
    std::string odom_dir = this->get_parameter("odom_dir").as_string();

    map_T_odom_last.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    map_T_odom_last.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    map_stamped_T_odom.header.frame_id = "map";
    map_stamped_T_odom.child_frame_id = "odom";
    map_stamped_T_odom.transform = tf2::toMsg(map_T_odom_last);

    kitti_utils = KittiUtils(data_dir,sequence,odom_dir);
    calib_data = kitti_utils.read_calib_file();
    //std::cout << calib_data[0].size() << std::endl;

    kitti_utils.get_point_cloud_files();
    kitti_utils.get_left_images();
    poses = kitti_utils.odom_pose();

    process_calib();

    std::string path = kitti_utils.file_names_image_gray_left_[0];
    Mat frame;
    frame = imread(path);
    image_height = frame.rows;
    image_width = frame.cols;

    handle_static_transform();
    

    timer_ = create_wall_timer(
    100ms, std::bind(&StreamerNode::on_timer_callback, this));
}

void StreamerNode::process_calib() {
    Eigen::MatrixXf P_rect_00 = Eigen::Map<Eigen::Matrix<double, 3, 4>>(calib_data["P0"].data()).cast<float>();
    Eigen::MatrixXf P_rect_10 = Eigen::Map<Eigen::Matrix<double, 3, 4>>(calib_data["P1"].data()).cast<float>();
   std::cout << P_rect_00 << std::endl;

    Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
    T1(0, 3) = P_rect_10(0, 3) / P_rect_10(0, 0);

    
    Tr = Eigen::Map<Eigen::Matrix<double, 3, 4>>(calib_data["Tr"].data()).cast<float>();
     
    camera_base_rotation <<
        Tr(0, 0), Tr(1, 0), Tr(2, 0), 0,
        Tr(0, 1), Tr(1, 1), Tr(2, 1), 0,
        Tr(0, 2), Tr(1, 2), Tr(2, 2), 0,
        0,0,0, 1;

    K_cam0 = P_rect_00.leftCols<3>();
    K_cam1 = P_rect_10.leftCols<3>();
}

void StreamerNode::publish_camera_info_msg(rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher,std::string camera_type) {
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header.frame_id = camera_type;
  camera_info_msg.header.stamp = this->get_clock()->now();
  camera_info_msg.height = image_height;
  camera_info_msg.width  = image_width;
  for (int i = 0; i < 9; ++i) {
      if(camera_type == "camera_left")
        camera_info_msg.k[i] = K_cam0(i / 3, i % 3);
      else
        camera_info_msg.k[i] = K_cam1(i / 3, i % 3);
  }
  publisher->publish(camera_info_msg);

}

// void StreamerNode::handle_map_to_odom_transform() {
//   geometry_msgs::msg::TransformStamped t;
//   rclcpp::Time now = this->get_clock()->now();
//   t.header.frame_id = "map";
//   t.child_frame_id = "camera_left";
//   t.header.stamp = now;

//   Eigen::Matrix4f pose = poses[file_index_];

//   Eigen::Vector3f translation_vector = pose.block<3, 1>(0, 3);
//   std::cout << "********" << translation_vector << std::endl;
//   Eigen::Matrix3f rotation_matrix = pose.block<3, 3>(0, 0);

//   tf2::Matrix3x3 tf2_rotation_matrix(
//         rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
//         rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
//         rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
//     );

//   // Convert tf2::Matrix3x3 to tf2::Quaternion
//   tf2::Quaternion quaternion;
//   tf2_rotation_matrix.getRotation(quaternion);

//   t.transform.translation.x = translation_vector[0];
//   t.transform.translation.y = translation_vector[1];
//   t.transform.translation.z = translation_vector[2];

//   t.transform.rotation.x = quaternion.x();
//   t.transform.rotation.y = quaternion.y();
//   t.transform.rotation.z = quaternion.z();
//   t.transform.rotation.w = quaternion.w();

//   tf_broadcaster_->sendTransform(t);
// }

// void StreamerNode::handle_velo_cam_transform() {
//   geometry_msgs::msg::TransformStamped t;
//   rclcpp::Time now = this->get_clock()->now();
//   t.header.frame_id = "camera_left";
//   t.child_frame_id = "velo_link";
//   t.header.stamp = now;

//   Eigen::Vector3f translation_vector = T_cam0_velo.block<3, 1>(0, 3);
  
//   Eigen::Matrix3f rotation_matrix = T_cam0_velo.block<3, 3>(0, 0);

//   tf2::Matrix3x3 tf2_rotation_matrix(
//         rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
//         rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
//         rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
//     );

//   // Convert tf2::Matrix3x3 to tf2::Quaternion
//   tf2::Quaternion quaternion;
//   tf2_rotation_matrix.getRotation(quaternion);

//   t.transform.translation.x = translation_vector[0];
//   t.transform.translation.y = translation_vector[1];
//   t.transform.translation.z = translation_vector[2];

//   t.transform.rotation.x = quaternion.x();
//   t.transform.rotation.y = quaternion.y();
//   t.transform.rotation.z = quaternion.z();
//   t.transform.rotation.w = quaternion.w();

//   tf_static_broadcaster_->sendTransform(t);
// }

void StreamerNode::on_timer_callback() {
    
    publish_odom_msg();
    publish_camera_info_msg(publisher_image_gray_left_info,"camera_left");
    publish_camera_info_msg(publisher_image_gray_right_info,"camera_right");
    convert_bin_to_pointcloud2();
    convert_image_to_msg(publisher_image_gray_left_,"camera_left");
    convert_image_to_msg(publisher_image_gray_right_,"camera_right");

    
    file_index_++;
}

void StreamerNode::convert_bin_to_pointcloud2() {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::string filePath = kitti_utils.file_names_point_cloud_[file_index_];
    std::fstream input(filePath, std::ios::in | std::ios::binary);
    if(!input.good()){
      RCLCPP_ERROR(this->get_logger(), "Could not read Velodyne's point cloud. Check your file path!");
      return;
      //exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    for (int i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud.push_back(point);
    }

    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "lidar_link";
    msg.header.stamp = now();

    publisher_point_cloud_->publish(msg);

} 

std::string StreamerNode::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void StreamerNode::convert_image_to_msg(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,std::string camera_type) {
    sensor_msgs::msg::Image msg;
    std::string path = kitti_utils.file_names_image_gray_left_[file_index_];
    Mat frame;
    frame = imread(path);
    if (frame.empty())                      // Check for invalid input
    {
        RCLCPP_ERROR(this->get_logger(), "Image does not exist. Check your files path!");
        rclcpp::shutdown();
    }

    msg.height = frame.rows;
    msg.width = frame.cols;
    std::string type = mat_type2encoding(frame.type());
    msg.encoding = type;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = camera_type;
    cam_time = this->get_clock()->now();
    msg.header.stamp = cam_time;
    publisher->publish(msg);
    

}

void StreamerNode::publish_odom_msg() {
  geometry_msgs::msg::PoseStamped msg;
  geometry_msgs::msg::TransformStamped map_odom_transform;
  map_odom_transform.header.stamp = cam_time;
  map_odom_transform.header.frame_id = "map";
  map_odom_transform.child_frame_id = "odom";
  msg.header.stamp = cam_time;
  msg.header.frame_id = "odom";
  Eigen::Matrix4f pose = poses[file_index_];

  auto& pose_matrix = camera_base_rotation * pose * camera_base_rotation.transpose();

  Eigen::Vector3f translation_vector = pose_matrix.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation_matrix = pose_matrix.block<3, 3>(0, 0);

  tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
        rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
        rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
    );

  // Convert tf2::Matrix3x3 to tf2::Quaternion
  tf2::Quaternion quaternion;
  tf2_rotation_matrix.getRotation(quaternion);

  msg.pose.position.x = translation_vector[0];
  msg.pose.position.y = translation_vector[1];
  msg.pose.position.z = translation_vector[2];

  map_odom_transform.transform.translation.x = translation_vector[0];
  map_odom_transform.transform.translation.y = translation_vector[1];
  map_odom_transform.transform.translation.z = translation_vector[2];

  map_odom_transform.transform.rotation = tf2::toMsg(quaternion);

  msg.pose.orientation.x = quaternion.x();
  msg.pose.orientation.y = quaternion.y();
  msg.pose.orientation.z = quaternion.z();
  msg.pose.orientation.w = quaternion.w();
  publisher_odometry_->publish(msg);

  tf_broadcaster_->sendTransform(map_odom_transform);

  publish_path_msg(msg);
}

void StreamerNode::publish_path_msg(geometry_msgs::msg::PoseStamped odom_msg) {
  geometry_msgs::msg::PoseStamped msg;
  msg.pose = odom_msg.pose;
  path_msg.poses.push_back(msg);
  path_msg.header.frame_id = "map";
  publisher_path_->publish(path_msg);
}

void StreamerNode::handle_static_transform() {
        //RCLCPP_INFO(this->get_logger(), "Publishing Static TF...");
 
        camera_lidar_transform.header.stamp = this->get_clock()->now();
        camera_lidar_transform.header.frame_id = "camera_left";
        camera_lidar_transform.child_frame_id = "lidar_link";

        // Set the translation from the extrinsic matrix
        camera_lidar_transform.transform.translation.x = Tr(0, 3);
        camera_lidar_transform.transform.translation.y = Tr(1, 3);
        camera_lidar_transform.transform.translation.z = Tr(2, 3);

        // Extract the rotation matrix and convert it to a quaternion
        tf2::Matrix3x3 rotation_matrix(
            Tr(0, 0), Tr(0, 1), Tr(0, 2),
            Tr(1, 0), Tr(1, 1), Tr(1, 2),
            Tr(2, 0), Tr(2, 1), Tr(2, 2));
        tf2::Quaternion quaternion;
        rotation_matrix.getRotation(quaternion);

        // Set the rotation in the transform
        camera_lidar_transform.transform.rotation = tf2::toMsg(quaternion);

        tf_static_broadcaster_->sendTransform(camera_lidar_transform);

        geometry_msgs::msg::TransformStamped base_camera_transform;
        base_camera_transform.header.stamp = this->get_clock()->now();
        base_camera_transform.header.frame_id = "base_link";
        base_camera_transform.child_frame_id = "camera_left";

        // Set the translation for base to camera link
        base_camera_transform.transform.translation.x = 0.0;
        base_camera_transform.transform.translation.y = 0.0;
        base_camera_transform.transform.translation.z = 0.0;

        // Convert the extrinsic matrix to a quaternion for the base to camera transform
        tf2::Matrix3x3 rotation_matrix_base(
            Tr(0, 0), Tr(1, 0), Tr(2, 0),
            Tr(0, 1), Tr(1, 1), Tr(2, 1),
            Tr(0, 2), Tr(1, 2), Tr(2, 2));
        rotation_matrix_base.getRotation(quaternion);

        // Set the rotation in the transform
        base_camera_transform.transform.rotation = tf2::toMsg(quaternion);

        tf_static_broadcaster_->sendTransform(base_camera_transform);

        geometry_msgs::msg::TransformStamped odom_base_transform;
        base_camera_transform.header.stamp = this->get_clock()->now();
        base_camera_transform.header.frame_id = "odom";
        base_camera_transform.child_frame_id = "base_link";

        // Set the translation for base to camera link
        base_camera_transform.transform.translation.x = 0.0;
        base_camera_transform.transform.translation.y = 0.0;
        base_camera_transform.transform.translation.z = 0.0;

        base_camera_transform.transform.rotation.x = 0.0;
        base_camera_transform.transform.rotation.y = 0.0;
        base_camera_transform.transform.rotation.z = 0.0;
        base_camera_transform.transform.rotation.w = 1.0;

        tf_static_broadcaster_->sendTransform(base_camera_transform);
        
    
    }

