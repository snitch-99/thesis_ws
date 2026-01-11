from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # We use the rtabmap_ros package from the rtabmap workspace
    # Since it's in a different workspace, get_package_share_directory might fail if not sourced.
    # We assume the user has sourced the rtabmap workspace.
    
    try:
        # Check if we can find rtabmap_launch
        rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
        rtabmap_launch_file = os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
    except Exception as e:
        print("Error: Could not find rtabmap_launch package. Ensure rtabmap workspace is sourced.")
        raise e

    return LaunchDescription([
        # Launch RTAB-Map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'frame_id': 'base_link',
                'subscribe_depth': 'true',
                'bg_color': '0 0 0',
                'use_sim_time': 'true',
                
                # Topics (remapped to our synced topics)
                'rgb_topic': '/camera/rgb_synced',
                'depth_topic': '/camera/depth_synced',
                'camera_info_topic': '/camera/camera_info_synced',
                'odom_topic': '/camera/odom_synced',
                
                # Synchronization (We do it manually in synced_broadcaster)
                'approx_sync': 'false', # Exact sync required since we produce exact timestamps
                'queue_size': '50',
                'wait_for_transform': '1.0',
                
                # Frames
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'visual_odometry': 'false', # We provide external odometry
                'icp_odometry': 'false',
                
                # QoS (Match synced_broadcaster)
                'qos': '1', # Reliable
                
                # RTAB-Map Args
                # --delete_db_on_start: Fresh map every time
                # --Optimizer/GravitySigma 0.3: Relax gravity constraint if needed
                'args': '--delete_db_on_start',
                
                # Visualization
                'rtabmap_viz': 'true',
                'rviz': 'false',
            }.items()
        )
    ])
