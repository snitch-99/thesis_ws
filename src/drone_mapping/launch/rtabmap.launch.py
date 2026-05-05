from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    try:
        rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
        rtabmap_launch_file = os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
    except Exception as e:
        print("Error: Could not find rtabmap_launch package. Ensure rtabmap workspace is sourced.")
        raise e

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'frame_id': 'base_link',
                'subscribe_depth': 'true',
                'bg_color': '0 0 0',
                'use_sim_time': 'true',

                # Topics (frame_id corrected by synced_broadcaster)
                'rgb_topic': '/camera/rgb_corrected',
                'depth_topic': '/camera/depth_corrected',
                'camera_info_topic': '/camera/camera_info_corrected',
                'odom_topic': '/ground_truth/odom',

                # RTAB-Map handles synchronization
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.1',
                'queue_size': '100',
                'wait_for_transform': '1.0',

                # Frames
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'visual_odometry': 'false',
                'icp_odometry': 'false',

                # QoS
                'qos': '1',

                # RTAB-Map Args
                'args': '--delete_db_on_start --Grid/RangeMax 15.0 --Grid/CellSize 0.01',
                'cloud_voxel_size': '0.0',
                'cloud_decimation': '1',

                # Visualization
                'rtabmap_viz': 'true',
                'rviz': 'false',
            }.items()
        )
    ])
