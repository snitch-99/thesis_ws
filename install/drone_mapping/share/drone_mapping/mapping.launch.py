import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_mapping',
            executable='data_collector',
            name='data_collector',
            output='screen',
        ),
        Node(
            package='drone_mapping',
            executable='position_control',
            name='position_control',
            output='screen',
        ),
    ])
