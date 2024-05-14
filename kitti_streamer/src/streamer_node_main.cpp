
#include <memory>
#include "kitti_streamer/streamer_node.h"


#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<StreamerNode>());

  rclcpp::shutdown();

  return 0;
}