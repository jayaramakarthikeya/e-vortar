cmake_minimum_required(VERSION 3.5)
project(kitti_streamer)
set(ALLOW_DUPLICATE_CUSTOM_TARGETS TRUE)


find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system) 
find_package(PCL COMPONENTS common REQUIRED)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(dependencies
    OpenCV
    geometry_msgs
    nav_msgs
    visualization_msgs
    pcl_conversions
    rcl_interfaces
    rclcpp
    rcutils
    sensor_msgs
    std_msgs
    tf2 
    tf2_ros
    tf2_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


set(ROS2_KITTI_STREAMER_SOURCES
  src/streamer_node.cpp
  src/kitti_utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${ROS2_KITTI_STREAMER_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(kitti_streamer_node
  src/streamer_node_main.cpp
)
ament_target_dependencies(kitti_streamer_node ${dependencies})


target_link_libraries(kitti_streamer_node
  ${PROJECT_NAME}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)



install(TARGETS
  ${PROJECT_NAME}
  kitti_streamer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)


ament_package()
