#include "../include/visual_odometry/visual_odometry.h"


VisualOdometry::VisualOdometry() : Node("visual_odometry") 
{
  auto node = rclcpp::Node::make_shared("visual_odometry");
  pub_point_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_follow_VO", 5);

   rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

  this->declare_parameter("loam_verbose_level",1);
  this->declare_parameter("reset_VO_to_identity",true);
  this->declare_parameter("remove_VO_outlier",0);
  this->declare_parameter("CLAHE",false);
  this->declare_parameter("visualize_optical_flow",false);
  this->declare_parameter("optical_flow_match",false);


  this->get_parameter("loam_verbose_level", verbose_level);
  this->get_parameter("reset_VO_to_identity", reset_VO_to_identity);
  this->get_parameter("remove_VO_outlier", remove_VO_outlier);
  this->get_parameter("CLAHE", CLAHE);
  this->get_parameter("visualize_optical_flow", visualize_optical_flow);
  this->get_parameter("optical_flow_match", optical_flow_match);

  tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr);

  sub_image00_ptr =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this,"camera/left/image_raw",rmw_qos_profile);
  sub_camera00_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this,"camera/left/camera_info",rmw_qos_profile);

  sub_lidar_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this,"lidar/top/point_cloud_raw",rmw_qos_profile);

  

  world_VOT_odom_last.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  world_VOT_odom_last.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  clahe = cv::createCLAHE(2.0);

  count = -1;

  
  image_util.print_result = false;
  image_util.visualize_result = false;
  image_util.detector_type = DetectorType::ShiTomasi;
  image_util.descriptor_type = DescriptorType::ORB;
  image_util.matcher_type = MatcherType::BF;
  image_util.selector_type = SelectType::KNN;
  image_util.remove_VO_outlier = remove_VO_outlier;
  image_util.optical_flow_match = optical_flow_match;
  image_util.print_result = true;
  image_util.visualize_result = false;

  images.clear();
  images.resize(2);
  keypoints.clear();
  keypoints.resize(2);
  descriptors.clear();
  descriptors.resize(2);
  matches.clear();
  if (optical_flow_match)
  {
    keypoints_2f.clear();
    keypoints_2f.resize(2);
  }

  point_cloud_utils.clear();
  point_cloud_utils.resize(2);
  point_cloud_utils[0].print_result = false;
  point_cloud_utils[0].downsample_grid_size = 5;
  point_cloud_utils[1].print_result = false;
  point_cloud_utils[1].downsample_grid_size = 5;

  for (j = 0; j < 3; ++j)
  {
    angles_0to1[j] = 0.0;
    t_0to1[j] = 0.0;
  }

  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;  // TODO: check the best solver
  // Reference: http://ceres-solver.org/nnls_solving.html#linearsolver. For small problems (a couple of hundred
  // parameters and a few thousand residuals) with relatively dense Jacobians, DENSE_QR is the method of choice In our
  // case, residual num is 1000~2000, but num of param is only 6 options.minimizer_progress_to_stdout = true;

  cam0_curr_T_cam0_last.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  cam0_curr_T_cam0_last.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  pubvisualOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/visual_odom_to_init", 100);
  pubvisualPath = this->create_publisher<nav_msgs::msg::Path>("/visual_odom_path", 100);
  visualPath.poses.clear();

  sync.reset(new Sync(MySyncPolicy(10),*sub_image00_ptr,*sub_camera00_ptr,*sub_lidar_ptr));
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), *sub_image00_ptr, *sub_camera00_ptr,
  //                                                  *sub_lidar_ptr);
  // message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
  //                                                       sensor_msgs::msg::PointCloud2> sync(20);
  sync->registerCallback(std::bind(&VisualOdometry::vi_callback,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  image_transport::ImageTransport it(node);
  pub_matches_viz = it.advertise("/visual_odometry/matches_visualization", 10);
  pub_depth_viz = it.advertise("/visual_odometry/depth_visualization", 10);
  pub_optical_flow_viz = it.advertise("/visual_odometry/optical_flow_visualization", 10);

  

  // timer_ = create_wall_timer(
  //   100ms, std::bind(&VisualOdometry::visual_timer_callback, this));
}

void VisualOdometry::visual_timer_callback() {
  process_static_transform();
}

void VisualOdometry::vi_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
              const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg) {
  i = count % 2;
  reset();
  process_static_transform();
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg,sensor_msgs::image_encodings::MONO8);
    processImage(cv_ptr->image);
  } catch(cv::Exception &e) {
    RCLCPP_INFO(this->get_logger(),e.what());
    return;
  }

  if(count==0) {
    
    setUpPointCloud(camera_info_msg);
  }

  pcl::fromROSMsg(*point_cloud_msg, point_cloud_pcl);
  try{
    processPointCloud(point_cloud_msg, point_cloud_pcl, true, false);
  } catch(cv::Exception &e) {
    RCLCPP_INFO(this->get_logger(),e.what());
    return;
  }
  

  if (count > 0)
  {
    solveNlsAll();  // result is cam0_curr_T_cam0_last, f2f odometry
                        // VO->solveNls2dOnly();
                        // VO->solveRANSAC();
  }

  VO2VeloAndBase(cam0_curr_T_cam0_last);
  try{
    publish();
  } catch(cv::Exception &e) {
    RCLCPP_INFO(this->get_logger(),e.what());
    return;
  }
  

  ++count;

}


void VisualOdometry::process_static_transform() {
  try {
    tf_stamped_lidar_T_cam0 = tf_buffer_ptr->lookupTransform("lidar_link","camera_left",this->get_clock()->now(),rclcpp::Duration(3.0));
  }
  catch(const tf2::TransformException & ex) {
      RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "lidar_link", "camera_left", ex.what());
          return;
  }
  
  velo_eigen_T_cam0 = tf2::transformToEigen(tf_stamped_lidar_T_cam0).cast<float>();
  tf2::fromMsg(tf_stamped_lidar_T_cam0.transform,velo_T_cam0);

  try{
    tf_stamped_cam0_T_base = tf_buffer_ptr->lookupTransform("camera_left","base_link",this->get_clock()->now(),rclcpp::Duration(3.0));
  }
  catch(const tf2::TransformException & ex) {
      RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "camera_left", "base_link", ex.what());
          return;
  }
  tf2::fromMsg(tf_stamped_cam0_T_base.transform,cam0_T_base);

  try {
    tf_stamped_base_T_odom = tf_buffer_ptr->lookupTransform("base_link","odom",this->get_clock()->now(),rclcpp::Duration(3.0));
  }
  catch(const tf2::TransformException & ex) {
      RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "base_link", "odom", ex.what());
          return;
  }
  
  tf2::fromMsg(tf_stamped_base_T_odom.transform,odom_T_base);

  RCLCPP_INFO(this->get_logger(),"Done!");

}

void VisualOdometry::reset()
{
  ++count;
  i = count % 2;
}

void VisualOdometry::VO2Cam0StartFrame() {
  if (count < 0)
    return;

  cam0_init_VOT_cam0_last = cam0_T_base * odom_T_base.inverse() * world_VOT_odom_last * odom_T_base  * cam0_T_base.inverse();

  if (count == 0)
    cam0_init_VOT_cam0_start = cam0_init_VOT_cam0_last;

  cam0_start_VOT_cam0_last = cam0_init_VOT_cam0_start.inverse() * cam0_init_VOT_cam0_last;

  cam0_start_eigen_VOT_cam0_last = tf2::transformToEigen(tf2::toMsg(cam0_start_VOT_cam0_last)).cast<float>();
}

void VisualOdometry::VO2VeloAndBase(const tf2::Transform &cam0_curr_VOT_cam0_last) {
  velo_last_VOT_velo_curr =
      velo_T_cam0 * cam0_curr_VOT_cam0_last.inverse() * velo_T_cam0.inverse();  // odom for velodyne
  // get T_base_last^base_curr (from VO)
  base_last_VOT_base_curr = cam0_T_base.inverse() * cam0_curr_VOT_cam0_last.inverse() * cam0_T_base;

  // get T_world^curr = T_last^curr * T_world^last
  geometry_msgs::msg::Transform temp = tf2::toMsg(base_last_VOT_base_curr);  // TODO: check better solution
  if (!std::isnan(temp.translation.x) and !std::isnan(temp.translation.y) and !std::isnan(temp.translation.z) and
      !std::isnan(temp.rotation.x) and !std::isnan(temp.rotation.y) and !std::isnan(temp.rotation.z) and
      !std::isnan(temp.rotation.w))                  // avoid nan at the first couple steps
    world_VOT_odom_last *= odom_T_base * base_last_VOT_base_curr;  // after update, last becomes the curr
  world_stamped_VOtf_odom.header.stamp = this->get_clock()->now();
  world_stamped_VOtf_odom.transform = tf2::toMsg(world_VOT_odom_last);
}

void VisualOdometry::processImage(const cv::Mat& img00)
{
  TicToc t_process_image;
  // std::string my_str = "my mat :";
  // my_str << img00;
  // std::cout << my_str << std::endl;

  RCLCPP_INFO(this->get_logger(),"image type = %d", img00.type());
  if (CLAHE)
    clahe->apply(img00, images[i]);
  else
    images[i] = img00;

  // if (keypoint_NMS)
  //     keypoints[i] = image_util.keyPointsNMS(image_util.detKeypoints(images[i]));
  // else
  keypoints[i] = image_util.detKeypoints(images[i]);  // TODO: might be optimizd

  if (!optical_flow_match)
    descriptors[i] = image_util.descKeypoints(keypoints[i], images[i]);

  if (verbose_level > 0)
  {
    //RCLCPP_INFO(this->get_logger(),"keypoint number = %ld \n", keypoints[i].size());
    //std::cout << "#^^$&#" << std::endl;
  }

  if (count > 0)
  { 
    RCLCPP_INFO(this->get_logger(),"match number \n");
    if (!optical_flow_match){
      std::cout << "Optical_before" << std::endl;
      matches = image_util.matchDescriptors(descriptors[1 - i],
                                            descriptors[i]);
      std::cout << "Optical_after" << std::endl;
                                            }  // first one is prev image, second one is curr image
    else {
      std::cout << "neufn3if_before" << std::endl;
      std::tie(keypoints_2f[1 - i], keypoints_2f[i], optical_flow_status) =
          image_util.calculateOpticalFlow(images[1 - i], images[i], keypoints[i]);
      std::cout << "k_before" << std::endl;
    }

      
    if (verbose_level > 0)
    {
      RCLCPP_INFO(this->get_logger(),"match number = %ld \n", matches.size());
    }
  }
  std::cout << "#^^$&#" << std::endl;
  RCLCPP_INFO(this->get_logger(),"Processing image took %f ms +++++\n", t_process_image.toc());
}

void VisualOdometry::setUpPointCloud(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg)
{
  TicToc t_set_up_point_cloud;
  point_cloud_utils[0].cam_T_velo = velo_eigen_T_cam0.matrix().inverse();
  point_cloud_utils[1].cam_T_velo = velo_eigen_T_cam0.matrix().inverse();

  // point from unrectified camera 00 to rectified camera 00
  for (j = 0; j < 9; ++j)
  {
    point_cloud_utils[0].rect0_T_cam(j / 3, j % 3) = camera_info_msg->r[j];  // TODO: optimize this code later
    point_cloud_utils[1].rect0_T_cam(j / 3, j % 3) = camera_info_msg->r[j];  // assume P doesn't change
  }

  // point from rectified camera 00 to image coordinate
  for (j = 0; j < 12; ++j)
  {
    point_cloud_utils[0].P_rect0(j / 4, j % 4) = camera_info_msg->p[j];  // TODO: optimize this code later
    point_cloud_utils[1].P_rect0(j / 4, j % 4) = camera_info_msg->p[j];  // assume P doesn't change
  }
  // std::cout << "\nP_rect0 = \n" << point_cloud_utils[0].P_rect0 << std::endl;

  RCLCPP_INFO(this->get_logger(),"Setting up point cloud took %f ms +++++\n", t_set_up_point_cloud.toc());
}

void VisualOdometry::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg,
                                       const pcl::PointCloud<pcl::PointXYZ>& point_cloud_pcl,
                                       const bool& visualize_depth, const bool& publish_point_cloud)
{
  TicToc t_process_point_cloud;

  point_cloud_3d_tilde = Eigen::MatrixXf::Ones(point_cloud_pcl.size(), 4);
  for (j = 0; j < point_cloud_pcl.size(); ++j)
  {
    point_cloud_3d_tilde(j, 0) = point_cloud_pcl.points[j].x;
    point_cloud_3d_tilde(j, 1) = point_cloud_pcl.points[j].y;
    point_cloud_3d_tilde(j, 2) = point_cloud_pcl.points[j].z;
  }
  point_cloud_utils[i].point_cloud_3d_tilde = point_cloud_3d_tilde;
  point_cloud_utils[i].projectPointCloud();
  point_cloud_utils[i].downsamplePointCloud();
  if (visualize_depth)
    point_cloud_utils[i].visualizeDepth(images[i]);  // uncomment this for depth visualization, but remember to reduce
                                                     // the bag playing speed too
  if (publish_point_cloud)
  {
    sensor_msgs::msg::PointCloud2 point_cloud_in_VO_msg = *point_cloud_msg;
    // ROS_INFO("point cloud frame id was %s", point_cloud_msg->header.frame_id.c_str());
    point_cloud_in_VO_msg.header.frame_id = "lidar_link";
    point_cloud_in_VO_msg.header.stamp = this->get_clock()->now();
    pub_point_cloud->publish(point_cloud_in_VO_msg);
  }

  RCLCPP_INFO(this->get_logger(),"Processing point cloud took %f ms +++++\n", t_process_point_cloud.toc());
}

void VisualOdometry::solveRANSAC()
{  // TODO: consider KLT case =>  it's not working with KLT
  TicToc t_ransac;

  std::vector<cv::Point2f> prev_pts, curr_pts;
  prev_pts.resize(matches.size());
  curr_pts.resize(matches.size());
  int j = 0;
  for (const auto& match : matches)
  {  // ~ n=1400 matches
    prev_pts[j] = keypoints[1 - i][match.queryIdx].pt;
    curr_pts[j] = keypoints[i][match.trainIdx].pt;
    ++j;
  }

  cv::Mat camera_matrix;
  cv::eigen2cv(point_cloud_utils[0].P_rect0, camera_matrix);
  // std::cout << camera_matrix << std::endl;
  camera_matrix = camera_matrix.colRange(0, 3);
  cv::Mat E = cv::findEssentialMat(prev_pts, curr_pts, camera_matrix, cv::RANSAC, 0.999, 1.0);
  // cv::Mat E = cv::findEssentialMat(prev_pts, curr_pts, camera_matrix, cv::LMEDS);
  cv::Mat R, t;
  cv::Mat mask, triangulated_points;
  int recover_result = cv::recoverPose(E, prev_pts, curr_pts, camera_matrix, R, t, 200.0f);
  // ROS_INFO("recover result is = %d", recover_result);
  // // ROS_INFO("det of R is %.4f", cv::determinant(R));
  // ROS_INFO("R is  \n %.4f, %.4f, %.4f\n %.4f, %.4f, %.4f \n %.4f, %.4f, %.4f \n",
  //                 R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
  //                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
  //                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

  tf2::Transform T;
  tf2::Matrix3x3 R_tf2(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0),
                       R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1),
                       R.at<double>(2, 2));
  T.setBasis(R_tf2);
  T.setOrigin(tf2::Vector3(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));
  tf2::Quaternion q_tf2 = T.getRotation();

  // ROS_INFO("R quaternion is %.4f, %.4f, %.4f, %.4f", q_tf2.getX(), q_tf2.getY(), q_tf2.getZ(), q_tf2.getW());

  // ROS_INFO("From RANSAC, axis angle = %.4f, %.4f, %.4f",
  //     q_tf2.getAxis().getX() * q_tf2.getAngle(),
  //     q_tf2.getAxis().getY() * q_tf2.getAngle(),
  //     q_tf2.getAxis().getZ() * q_tf2.getAngle()
  // );
  RCLCPP_INFO(this->get_logger(),"From RANSAC axis = %.4f, %.4f, %.4f, and angle = %.4f", q_tf2.getAxis().getX(), q_tf2.getAxis().getY(),
           q_tf2.getAxis().getZ(), q_tf2.getAngle());
  RCLCPP_INFO(this->get_logger(),"Froma RANSAC, t = %.4f, %.4f, %.4f", t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));
  // std::cout << "t = " << t << std::endl;

  // cam0_curr_T_cam0_last.setRotation(q_tf2);
  // float f2f_distance = std::sqrt(
  //     std::pow(cam0_curr_T_cam0_last.getOrigin().getX(), 2) +
  //     std::pow(cam0_curr_T_cam0_last.getOrigin().getY(), 2) +
  //     std::pow(cam0_curr_T_cam0_last.getOrigin().getZ(), 2)
  // );
  // cam0_curr_T_cam0_last.setOrigin(tf2::Vector3(
  //     f2f_distance * t.at<double>(0,0),
  //     f2f_distance * t.at<double>(1,0),
  //     f2f_distance * t.at<double>(2,0)
  // ));

  RCLCPP_INFO(this->get_logger(),"RANSAC VO took %f ms +++++\n", t_ransac.toc());
}

void VisualOdometry::solveNlsAll()
{
  TicToc t_nls_all;

  ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
  ceres::Problem problem;

  if (reset_VO_to_identity)
  {
    for (j = 0; j < 3; ++j)
    {
      angles_0to1[j] = 0.0;
      t_0to1[j] = 0.0;
    }
  }
  // else
  // {
  //   // init from LO
  //   t_0to1[0] = vloam_tf->cam0_curr_LOT_cam0_prev.getOrigin().getX();
  //   t_0to1[1] = vloam_tf->cam0_curr_LOT_cam0_prev.getOrigin().getY();
  //   t_0to1[2] = vloam_tf->cam0_curr_LOT_cam0_prev.getOrigin().getZ();
  //   angles_0to1[0] = vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAxis().getX() *
  //                    vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAngle();
  //   angles_0to1[1] = vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAxis().getY() *
  //                    vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAngle();
  //   angles_0to1[2] = vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAxis().getZ() *
  //                    vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAngle();
  // }

  int prev_pt_x, prev_pt_y, curr_pt_x, curr_pt_y;
  int counter33 = 0, counter32 = 0, counter23 = 0, counter22 = 0;
  int match_num = (optical_flow_match) ? keypoints_2f[i].size() : matches.size();
  for (int j = 0; j < match_num; ++j)
  {  // ~ n=1400 matches

    if (!optical_flow_match)
    {
      prev_pt_x = keypoints[1 - i][matches[j].queryIdx].pt.x;
      prev_pt_y = keypoints[1 - i][matches[j].queryIdx].pt.y;
      curr_pt_x = keypoints[i][matches[j].trainIdx].pt.x;
      curr_pt_y = keypoints[i][matches[j].trainIdx].pt.y;
    }
    else
    {
      if (optical_flow_status[j] == 1)
      {
        prev_pt_x = keypoints_2f[1 - i][j].x;
        prev_pt_y = keypoints_2f[1 - i][j].y;
        curr_pt_x = keypoints_2f[i][j].x;
        curr_pt_y = keypoints_2f[i][j].y;
      }
      else
        continue;
    }

    if (remove_VO_outlier > 0)
    {
      if (std::pow(prev_pt_x - curr_pt_x, 2) + std::pow(prev_pt_y - curr_pt_y, 2) >
          remove_VO_outlier * remove_VO_outlier)
        continue;
    }

    depth0 = point_cloud_utils[1 - i].queryDepth(prev_pt_x, prev_pt_y);
    depth1 = point_cloud_utils[i].queryDepth(curr_pt_x, curr_pt_y);

    // if (std::abs(depth0 - depth1) > 3.0)
    //     continue;

    // if (depth0 > 0.5 and depth1 > 0.5) {
    //     point_3d_image0_0 << prev_pt_x*depth0, prev_pt_y*depth0, depth0;
    //     point_3d_image0_1 << curr_pt_x*depth1, curr_pt_y*depth1, depth1;

    //     point_3d_rect0_0 =
    //     (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0); point_3d_rect0_1
    //     = (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1);

    //     // assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);
    //     // assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

    //     ceres::CostFunction* cost_function = vloam::CostFunctor33::Create(
    //             static_cast<double>(point_3d_rect0_0(0)),
    //             static_cast<double>(point_3d_rect0_0(1)),
    //             static_cast<double>(point_3d_rect0_0(2)),
    //             static_cast<double>(point_3d_rect0_1(0)),
    //             static_cast<double>(point_3d_rect0_1(1)),
    //             static_cast<double>(point_3d_rect0_1(2))
    //     );
    //     problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
    //     ++counter33;
    // }
    // else if (depth0 > 0.5 and depth1 <= 0.5) {
    if (depth0 > 0)
    {
      point_3d_image0_0 << prev_pt_x * depth0, prev_pt_y * depth0, depth0;
      point_3d_image0_1 << curr_pt_x, curr_pt_y, 1.0f;

      point_3d_rect0_0 =
          (point_cloud_utils[1 - i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0);  // assume
                                                                                                          // point_cloud_utils[i].P_rect0.col(3)
                                                                                                          // is zero
      point_3d_rect0_1 =
          (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1);  // assume
                                                                                                      // point_cloud_utils[i].P_rect0.col(3)
                                                                                                      // is zero

      // assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);

      ceres::CostFunction* cost_function = CostFunctor32::Create(
          static_cast<double>(point_3d_rect0_0(0)), static_cast<double>(point_3d_rect0_0(1)),
          static_cast<double>(point_3d_rect0_0(2)),
          static_cast<double>(point_3d_rect0_1(0)) / static_cast<double>(point_3d_rect0_1(2)),
          static_cast<double>(point_3d_rect0_1(1)) / static_cast<double>(point_3d_rect0_1(2)));
      problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
      ++counter32;
    }

    // else if (depth0 <= 0.5 and depth1 > 0.5) {
    // // if (depth1 > 0) {
    //     point_3d_image0_0 << prev_pt_x, prev_pt_y, 1.0f;
    //     point_3d_image0_1 << curr_pt_x*depth1, curr_pt_y*depth1, depth1;

    //     point_3d_rect0_0 =
    //     (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0); // assume
    //     point_cloud_utils[i].P_rect0.col(3) is zero point_3d_rect0_1 =
    //     (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1); // assume
    //     point_cloud_utils[i].P_rect0.col(3) is zero

    // //     // assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

    //     ceres::CostFunction* cost_function = vloam::CostFunctor23::Create(
    //             static_cast<double>(point_3d_rect0_0(0)) / static_cast<double>(point_3d_rect0_0(2)),
    //             static_cast<double>(point_3d_rect0_0(1)) / static_cast<double>(point_3d_rect0_0(2)),
    //             static_cast<double>(point_3d_rect0_1(0)),
    //             static_cast<double>(point_3d_rect0_1(1)),
    //             static_cast<double>(point_3d_rect0_1(2))
    //     );
    //     problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
    //     ++counter23;
    // }
    else
    {
      // if (depth0 < 0 and depth1 < 0) {
      point_3d_image0_0 << prev_pt_x, prev_pt_y, 1.0f;
      point_3d_image0_1 << curr_pt_x, curr_pt_y, 1.0f;

      point_3d_rect0_0 =
          (point_cloud_utils[1 - i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0);  // assume
                                                                                                          // point_cloud_utils[i].P_rect0.col(3)
                                                                                                          // is zero
      point_3d_rect0_1 =
          (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1);  // assume
                                                                                                      // point_cloud_utils[i].P_rect0.col(3)
                                                                                                      // is zero

      ceres::CostFunction* cost_function = CostFunctor22::Create(
          static_cast<double>(point_3d_rect0_0(0)) / static_cast<double>(point_3d_rect0_0(2)),
          static_cast<double>(point_3d_rect0_0(1)) / static_cast<double>(point_3d_rect0_0(2)),
          static_cast<double>(point_3d_rect0_1(0)) / static_cast<double>(point_3d_rect0_1(2)),
          static_cast<double>(point_3d_rect0_1(1)) / static_cast<double>(point_3d_rect0_1(2)));
      problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
      ++counter22;
    }
  }

  // ROS_INFO("counter33 = %d", counter33);
  RCLCPP_INFO(this->get_logger(),"counter32 = %d", counter32);
  // ROS_INFO("counter23 = %d", counter33);
  RCLCPP_INFO(this->get_logger(),"counter22 = %d", counter22);

  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";

  cam0_curr_T_cam0_last.setOrigin(tf2::Vector3(t_0to1[0], t_0to1[1], t_0to1[2]));
  angle = std::sqrt(std::pow(angles_0to1[0], 2) + std::pow(angles_0to1[1], 2) + std::pow(angles_0to1[2], 2));
  cam0_curr_q_cam0_last.setRotation(
      tf2::Vector3(angles_0to1[0] / angle, angles_0to1[1] / angle, angles_0to1[2] / angle), angle);
  cam0_curr_T_cam0_last.setRotation(cam0_curr_q_cam0_last);

  RCLCPP_INFO(this->get_logger(),"angles_0to1 = (%.4f, %.4f, %.4f)", angles_0to1[0], angles_0to1[1], angles_0to1[2]);
  // ROS_INFO("q_0to1 = (%.4f, %.4f, %.4f, %.4f)", cam0_curr_q_cam0_last.getX(), cam0_curr_q_cam0_last.getY(),
  // cam0_curr_q_cam0_last.getZ(), cam0_curr_q_cam0_last.getW());
  RCLCPP_INFO(this->get_logger(),"From nllsq axis = %.4f, %.4f, %.4f, and angle = %.4f", cam0_curr_q_cam0_last.getAxis().getX(),
           cam0_curr_q_cam0_last.getAxis().getY(), cam0_curr_q_cam0_last.getAxis().getZ(),
           cam0_curr_q_cam0_last.getAngle());
  RCLCPP_INFO(this->get_logger(),"t_0to1 = (%.4f, %.4f, %.4f)", t_0to1[0], t_0to1[1], t_0to1[2]);

  // ROS_INFO("From LM axis = %.4f, %.4f, %.4f, and angle = %.4f",
  //     cam0_curr_q_cam0_last.getAxis().getX(),
  //     cam0_curr_q_cam0_last.getAxis().getY(),
  //     cam0_curr_q_cam0_last.getAxis().getZ(),
  //     cam0_curr_q_cam0_last.getAngle()
  // );

  // return cam0_curr_T_cam0_last;

  RCLCPP_INFO(this->get_logger(),"Nonlinear Least square (ALL) VO took %f ms +++++\n", t_nls_all.toc());
}

void VisualOdometry::publish()
{
  visualOdometry.header.frame_id = "map";
  visualOdometry.child_frame_id = "visual_odom";
  visualOdometry.header.stamp = this->get_clock()->now();                                // image_msg->header.stamp;

  Eigen::Quaterniond q_wodom_curr(world_VOT_odom_last.getRotation());  // wodom to cam
  Eigen::Vector3d t_wodom_curr(world_VOT_odom_last.getOrigin());       // wodom to cam
  visualOdometry.pose.pose.orientation.x = q_wodom_curr.x();
  visualOdometry.pose.pose.orientation.y = q_wodom_curr.y();
  visualOdometry.pose.pose.orientation.z = q_wodom_curr.z();
  visualOdometry.pose.pose.orientation.w = q_wodom_curr.w();
  visualOdometry.pose.pose.position.x = t_wodom_curr.x();
  visualOdometry.pose.pose.position.y = t_wodom_curr.y();
  visualOdometry.pose.pose.position.z = t_wodom_curr.z();
  pubvisualOdometry->publish(visualOdometry);
  // ROS_INFO("publish visualOdometry x = %f and %f", vloam_tf->world_VOT_base_last.getOrigin().x(), t_wodom_curr.x());

  // vloam_tf->world_VOT_base_last.setOrigin(tf2::Vector3(
  //     t_wodom_curr.x(),
  //     t_wodom_curr.y(),
  //     t_wodom_curr.z()
  // ));
  // vloam_tf->world_VOT_base_last.setRotation(tf2::Quaternion(
  //     q_wodom_curr.x(),
  //     q_wodom_curr.y(),
  //     q_wodom_curr.z(),
  //     q_wodom_curr.w()
  // ));

  geometry_msgs::msg::PoseStamped visualPose;
  visualPose.header = visualOdometry.header;
  visualPose.pose = visualOdometry.pose.pose;
  visualPath.header.stamp = visualOdometry.header.stamp;
  visualPath.header.frame_id = "map";
  visualPath.poses.push_back(visualPose);
  pubvisualPath->publish(visualPath);

  if (verbose_level > 0 and count > 1)
  {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    cv::Mat match_image =
        image_util.visualizeMatches(images[1 - i], images[i], keypoints[1 - i], keypoints[i], matches);
    matches_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, match_image);
    pub_matches_viz.publish(matches_viz_cvbridge.toImageMsg());
  }

  if (verbose_level > 0 and count > 0)
  {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    cv::Mat depth_image = point_cloud_utils[i].visualizeDepth(images[i]);
    depth_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, depth_image);
    pub_depth_viz.publish(depth_viz_cvbridge.toImageMsg());
  }

  if (verbose_level > 0 and count > 0 and visualize_optical_flow)
  {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();

    cv::Mat optical_flow_image;
    if (optical_flow_match)
      optical_flow_image =
          image_util.visualizeOpticalFlow(images[i], keypoints_2f[1 - i], keypoints_2f[i], optical_flow_status);
    else
      optical_flow_image = image_util.visualizeOpticalFlow(images[i], keypoints[1 - i], keypoints[i], matches);

    optical_flow_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, optical_flow_image);
    pub_optical_flow_viz.publish(optical_flow_viz_cvbridge.toImageMsg());

  }
  std::cout << "DOne!" << std::endl;
}