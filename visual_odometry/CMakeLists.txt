cmake_minimum_required(VERSION 3.5)
project(visual_odometry)
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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(kitti_streamer REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) 
find_package(PCL COMPONENTS common REQUIRED)
find_package(Ceres REQUIRED PATHS "/usr/local/lib/cmake/Ceres/")
find_package(message_filters REQUIRED)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
    tf2_geometry_msgs
    tf2_msgs
    image_transport
    cv_bridge
    kitti_streamer
    message_filters
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


set(ROS2_VISUAL_ODOMETRY_SOURCES
  src/image_util.cpp
  src/point_cloud_util.cpp
  src/visual_odometry.cpp
)

add_library(${PROJECT_NAME} SHARED ${ROS2_VISUAL_ODOMETRY_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(visual_odom_node
  src/visual_odometry_main.cpp
)
ament_target_dependencies(visual_odom_node ${dependencies})


target_link_libraries(visual_odom_node
  ${PROJECT_NAME}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)



install(TARGETS
  ${PROJECT_NAME}
  visual_odom_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
