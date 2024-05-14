

#ifndef KITTI_STREAMER_KITTI_UTILS_H_
#define KITTI_STREAMER_KITTI_UTILS_H_

#include <iostream>
#include <vector>
#include <filesystem>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <unordered_map>
#include <sstream>
#include <Eigen/Dense>

namespace fs = boost::filesystem;


class KittiUtils {

    public:
        KittiUtils() = default;
        KittiUtils(std::string data_dir,std::string sequence_no,std::string odom_dir);
        void get_left_images();
        void get_files(std::string directory,std::vector<std::string> extensions,std::vector<std::string>& files_list);
        void get_right_images();
        void get_point_cloud_files();
        std::vector<Eigen::Matrix4f> odom_pose();
        std::unordered_map<std::string, std::vector<double>> read_calib_file();


        std::vector<std::string> file_names_point_cloud_;
        std::vector<std::string> file_names_image_gray_left_;
        std::vector<std::string> file_names_image_gray_right_;

        std::string lidar_dir;
        std::string left_cam_sequence_dir;
        std::string right_cam_sequence_dir;
        std::string calib_file;
        std::string time_file;
        std::string odom_file_;
        std::string kitti_sequence_dir;
};



#endif