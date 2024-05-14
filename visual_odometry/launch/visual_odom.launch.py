from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    streamer_config = '/home/karthik/vloam/vloam_ws/src/kitti_streamer/config/params.yaml'
    visual_odom_config = '/home/karthik/vloam/vloam_ws/src/visual_odometry/config/params.yaml'
    return LaunchDescription([
        Node(
            package='kitti_streamer',
            namespace='',
            executable='kitti_streamer_node',
            name='streamer_node',
            parameters=[streamer_config]
        ),
        Node(
            package='visual_odometry',
            namespace='',
            executable='visual_odom_node',
            name='visual_odometry',
            parameters=[visual_odom_config]
        )
    ])  