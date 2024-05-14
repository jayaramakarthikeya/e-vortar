

#include "kitti_streamer/kitti_utils.h"

#include "rclcpp/rclcpp.hpp"

KittiUtils::KittiUtils(std::string data_dir,std::string sequence_no,std::string odom_file) {
    std::cout << "&&&&" << std::endl;
    kitti_sequence_dir = data_dir + "sequences/" + sequence_no + "/";
    std::cout << kitti_sequence_dir << std::endl;
    if (odom_file != "") odom_file_ = odom_file + "poses/" + sequence_no + ".txt";
    left_cam_sequence_dir = data_dir + "sequences/" + sequence_no + "/image_0/";
    right_cam_sequence_dir = data_dir + "sequences/" + sequence_no + "/image_1/";
    lidar_dir = data_dir + "sequences/" + sequence_no + "/velodyne/";
    calib_file = kitti_sequence_dir + "calib.txt";
    time_file = kitti_sequence_dir + "times.txt";
}

void KittiUtils::get_files(std::string directory,std::vector<std::string> extensions,std::vector<std::string>& files_list) {
    fs::path files_path(directory);
    if(fs::exists(files_path) && fs::is_directory(files_path)) {
        for (auto const & entry : fs::recursive_directory_iterator(files_path))
        {
            for (std::string ext:extensions) {
                if (fs::is_regular_file(entry) && entry.path().extension() == ext) {
                    files_list.emplace_back(directory+entry.path().filename().string());
                }
            }
        }
    }

    std::sort(files_list.begin(),files_list.end());
}

void KittiUtils::get_left_images() {
    std::vector<std::string> image_extensions ={".jpg", ".jpeg", ".png", ".bmp", ".gif"};
    get_files(left_cam_sequence_dir,image_extensions,file_names_image_gray_left_);

}

void KittiUtils::get_right_images() {
    std::vector<std::string> image_extensions ={".jpg", ".jpeg", ".png", ".bmp", ".gif"};
    get_files(right_cam_sequence_dir,image_extensions,file_names_image_gray_right_);
}

void KittiUtils::get_point_cloud_files() {
    std::vector<std::string> lidar_extensions ={".bin"};
    get_files(lidar_dir,lidar_extensions,file_names_point_cloud_);
}

std::unordered_map<std::string, std::vector<double>> KittiUtils::read_calib_file(){
    std::unordered_map<std::string, std::vector<double>> data;
    fs::path filepath(calib_file);
    if (fs::exists(filepath)) {
        std::ifstream file(filepath.string());
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                std::string key, value_str;
                if (line.find(':') != std::string::npos) {
                    std::getline(iss, key, ':');
                    std::getline(iss, value_str);
                } else {
                    std::getline(iss, key, ' ');
                    std::getline(iss, value_str);
                }

                std::istringstream value_iss(value_str);
                std::vector<double> values;
                double value;
                while (value_iss >> value) {
                    values.push_back((double)value);
                }
                data[key] = values;
                //std::cout << data[key][0] << std::endl;
            }
            file.close();
        } else {
            std::cerr << "Unable to open file: " << filepath << std::endl;
        }
    } else {
        std::cerr << "File not found: " << filepath << std::endl;
    }

    //std::cout << data.at("P0")[0] << std::endl;
    return data;
}

std::vector<Eigen::Matrix4f> KittiUtils::odom_pose() {
    std::vector<Eigen::Matrix4f> homogenous_matrix_arr;
    fs::path odom_dir_path(odom_file_);

    if (fs::exists(odom_dir_path)) {
        std::ifstream file(odom_file_);
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                std::vector<double> transformation_data;
                std::istringstream iss(line);
                double val;
                while (iss >> val) {
                    transformation_data.push_back(val);
                }

                Eigen::Matrix4f homogenous_matrix = Eigen::Matrix4f::Identity();
                homogenous_matrix.row(0) << transformation_data[0], transformation_data[1], transformation_data[2], transformation_data[3];
                homogenous_matrix.block<1, 4>(1, 0) << transformation_data[4], transformation_data[5], transformation_data[6], transformation_data[7];
                homogenous_matrix.block<1, 4>(2, 0) << transformation_data[8], transformation_data[9], transformation_data[10], transformation_data[11];
                homogenous_matrix.row(3) << 0 , 0, 0, 1;

                homogenous_matrix_arr.push_back(homogenous_matrix);
            }
            file.close();
        } else {
            std::cerr << "Unable to open file: " << odom_file_ << std::endl;
        }
    } else {
        std::cerr << "Odom directory not found: " << odom_file_ << ", Ground truth(Odometry) is available for only 10 sequences in KITTI. Stopping the process." << std::endl;
    }

    return homogenous_matrix_arr;
}
