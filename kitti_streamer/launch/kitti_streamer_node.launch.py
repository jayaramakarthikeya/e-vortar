from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = '/home/karthik/vloam/vloam_ws/src/kitti_streamer/config/params.yaml'
    print(config)
    return LaunchDescription([
        Node(
            package='kitti_streamer',
            namespace='',
            executable='kitti_streamer_node',
            name='streamer_node',
            parameters=[config]
        )
    ])      