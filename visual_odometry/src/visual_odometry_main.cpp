
#include <memory>
#include "../include/visual_odometry/visual_odometry.h"


#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<VisualOdometry>());

  rclcpp::shutdown();

  return 0;
}