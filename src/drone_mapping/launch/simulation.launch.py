import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, AppendEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    home_dir = os.environ.get('HOME')
    px4_dir = os.path.join(home_dir, 'PX4-Autopilot')
    qgc_path = os.path.join(home_dir, 'QGroundControl-x86_64.AppImage')
    drone_mapping_share = get_package_share_directory('drone_mapping')
    models_path = os.path.join(drone_mapping_share, 'models', 'entities')

    # Env Vars
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # --- DEFINITIONS ---

    # 1. QGroundControl
    qgc_launch = ExecuteProcess(
        cmd=[qgc_path],
        output='screen'
    )

    # 2. PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=PX4 SITL', '--', 'bash', '-c',
            'make px4_sitl gz_x500_depth; exec bash'
        ],
        cwd=px4_dir,
        output='screen'
    )

    mav_config = os.path.join(drone_mapping_share, 'config', 'mavros_config.yaml')

    # 3. MAVROS
    mavros_launch = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=MAVROS', '--', 'bash', '-c',
            f'ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 config_yaml:={mav_config}; exec bash'
        ],
        output='screen'
    )

    # 4. Spawning Rock (Target for Orbit)
    spawn_rock = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rock',
            '-x', '5.0',
            '-y', '-5.0',
            '-z', '0.5',
            '-file', os.path.join(models_path, 'rock', 'model.sdf')
        ],
        output='screen'
    )

    # 5. Traversability Node
    traversability_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Traversability Node', '--', 'bash', '-c',
            'ros2 run drone_mapping traversability; exec bash'
        ],
        output='screen'
    )

    # 6. Mavros Control Node
    mavros_control_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Mavros Control Node', '--', 'bash', '-c',
            'ros2 run drone_mapping mavros_control; exec bash'
        ],
        output='screen'
    )

    # 7. Bridge ROS-Gazebo Topics
    # Bridge Depth Image and Camera Info
    # Note: Using fully qualified Gazebo topic names provided by user/inspection
    # 8. Bridge ROS-Gazebo Topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/depth_camera', '/camera/depth'),
            ('/camera_info', '/camera/camera_info')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 9. Synced Broadcaster (TF + Image Forwarding)
    synced_broadcaster = Node(
        package='drone_mapping',
        executable='synced_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    # 10. Depth to Point Cloud Node
    depth_to_pointcloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='depth_to_pointcloud',
        parameters=[{'use_sim_time': True}],
        output='screen',
        remappings=[
            ('image_rect', '/camera/depth_synced'),
            ('camera_info', '/camera/camera_info_synced'),
            ('points', '/camera/points')
        ]
    )

    # 11. Rosbag Recorder (Synced Packet)
    rosbag_recorder = Node(
        package='drone_mapping',
        executable='rosbag_recorder',
        output='screen'
    )

    # --- SEQUENCE ---
    # T=0s: QGC (Start immediately)
    
    # T=5s: PX4 (Wait for QGC)
    delayed_px4 = TimerAction(period=5.0, actions=[px4_sitl])

    # T=12s: MAVROS (Wait for PX4/Gazebo to init)
    delayed_mavros = TimerAction(period=12.0, actions=[mavros_launch])

    # T=15s: Spawn Rock (Wait for Gazebo)
    delayed_rock = TimerAction(period=15.0, actions=[spawn_rock])

    # T=18s: Traversability Node (Wait for MAVROS)
    delayed_traversability = TimerAction(period=20.0, actions=[traversability_node])

    # T=22s: Mavros Control Node (Last)
    delayed_control = TimerAction(period=24.0, actions=[mavros_control_node])

    return LaunchDescription([
        set_gz_resource_path,
        qgc_launch,
        delayed_px4,
        delayed_mavros,
        delayed_rock,
        bridge,
        delayed_traversability,
        delayed_control,
        synced_broadcaster,
        depth_to_pointcloud,
        rosbag_recorder,

    ])
