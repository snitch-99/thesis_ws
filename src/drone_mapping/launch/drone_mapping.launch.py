import json
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, AppendEnvironmentVariable
from launch_ros.actions import Node

# ──────────────────────────────────────────────
# EXPERIMENT PARAMETERS
# ──────────────────────────────────────────────
# Change NUM_ROCKS (1-10) to select the scene.
# Survey area: center=(5.0, -5.0), radius=8m, fixed across all scenes.
# Min inter-rock spacing: 0.5m. All rocks spawn at z=0.0 (Blender-centred meshes).
NUM_ROCKS = 10
TRIAL_ID  = 3

HEADLESS_MODE = True

# ──────────────────────────────────────────────
# ROCK SPAWN CONFIGS
# All 10 positions are inside the 8m survey circle centered on (5, -5).
# Uncomment/comment individual entries below to manually tune scenes if needed.
## ──────────────────────────────────────────────
# ── Multi-rock scene config (original) ────────────────────────────────────────
ROCK_CONFIGS = [
    # Scene 0
    {'name': 'rock1',  'model': 'rock',    'x':  5.0, 'y':  -5.0, 'z': 0.55, 'R': 0.0,  'P': 0.0, 'Y': 0.0},
    # Scene 1
    {'name': 'rock2',  'model': 'rock2',   'x':  1.0, 'y':  -1.5, 'z': 0.8,  'R': 0.0,  'P': 0.0, 'Y': 0.0},
    # Scene 2
    {'name': 'rock3',  'model': 'rock3',   'x':  0.0, 'y':  -4.2, 'z': 0.3,  'R': 0.0,  'P': 0.0, 'Y': 0.0},
    # Scene 3
    {'name': 'rock4',  'model': 'rock4',   'x':  7.8, 'y':  -7.5, 'z': 0.6,  'R': 0.0,  'P': 0.0, 'Y': 0.0},
    # Scene 4
    {'name': 'rock5',  'model': 'rock5',   'x':  8.0, 'y':  -2.5, 'z': 0.20, 'R': 1.57, 'P': 0.0, 'Y': 0.0},
    # Scene 5
    {'name': 'rock6',  'model': 'rock6',   'x':  3.0, 'y':  -8.5, 'z': 0.25, 'R': 1.57, 'P': 0.0, 'Y': 0.0},
    # Scene 6
    {'name': 'rock7',  'model': 'rock7',   'x':  6.5, 'y':  -9.5, 'z': 0.3,  'R': 1.57, 'P': 0.0, 'Y': 0.0},
    # Scene 7
    {'name': 'rock8',  'model': 'rock8',   'x':  0.5, 'y':  -7.5, 'z': 0.3,  'R': 0.0,  'P': 0.0, 'Y': 0.0},
    # Scene 8
    {'name': 'rock9',  'model': 'rock9',   'x':  4.0, 'y':  -1.0, 'z': 0.2,  'R': 0.0,  'P': 0.0, 'Y': 0.0},
    # Scene 9
    {'name': 'rock10', 'model': 'rock10',  'x':  9.5, 'y':  -4.8, 'z': 0.0,  'R': 0.0,  'P': 0.0, 'Y': 0.0},
    # Terrain
    {'name': 'terrain', 'model': 'terrain', 'x': 5.0, 'y':  -5.0, 'z': -0.8, 'R': 0.0,  'P': 0.0, 'Y': 0.0},
]

# ── Individual rock configs — each spawned at (5, -5) for isolated scans ──────
# Uncomment ONE block at a time. Make sure NUM_ROCKS = 1.

# ROCK 1
#ROCK_CONFIGS = [{'name': 'rock1',  'model': 'rock',   'x': 5.0, 'y': -5.0, 'z': 0.55, 'R': 0.0,  'P': 0.0, 'Y': 0.0}]

# ROCK 2
#ROCK_CONFIGS = [{'name': 'rock2',  'model': 'rock2',  'x': 5.0, 'y': -5.0, 'z': 0.8,  'R': 0.0,  'P': 0.0, 'Y': 0.0}]

# ROCK 3
#ROCK_CONFIGS = [{'name': 'rock3',  'model': 'rock3',  'x': 5.0, 'y': -5.0, 'z': 0.3,  'R': 0.0,  'P': 0.0, 'Y': 0.0}]

# ROCK 4
#ROCK_CONFIGS = [{'name': 'rock4',  'model': 'rock4',  'x': 5.0, 'y': -5.0, 'z': 0.6,  'R': 0.0,  'P': 0.0, 'Y': 0.0}]

# ROCK 5
#ROCK_CONFIGS = [{'name': 'rock5',  'model': 'rock5',  'x': 5.0, 'y': -5.0, 'z': 0.20, 'R': 1.57, 'P': 0.0, 'Y': 0.0}]

# ROCK 6
#ROCK_CONFIGS = [{'name': 'rock6',  'model': 'rock6',  'x': 5.0, 'y': -5.0, 'z': 0.25, 'R': 1.57, 'P': 0.0, 'Y': 0.0}]

# ROCK 7
#ROCK_CONFIGS = [{'name': 'rock7',  'model': 'rock7',  'x': 5.0, 'y': -5.0, 'z': 0.3,  'R': 1.57, 'P': 0.0, 'Y': 0.0}]

# ROCK 8
#ROCK_CONFIGS = [{'name': 'rock8',  'model': 'rock8',  'x': 5.0, 'y': -5.0, 'z': 0.3,  'R': 0.0,  'P': 0.0, 'Y': 0.0}]

# ROCK 9
#ROCK_CONFIGS = [{'name': 'rock9',  'model': 'rock9',  'x': 5.0, 'y': -5.0, 'z': 0.2,  'R': 0.0,  'P': 0.0, 'Y': 0.0}]

# ROCK 10
#ROCK_CONFIGS = [{'name': 'rock10', 'model': 'rock10', 'x': 5.0, 'y': -5.0, 'z': 0.0,  'R': 0.0,  'P': 0.0, 'Y': 0.0}]

def generate_launch_description():

    # Directories
    home_dir = os.environ.get('HOME')
    px4_dir = os.path.join(home_dir, 'PX4-Autopilot')
    qgc_path = os.path.join(home_dir, 'QGroundControl-x86_64.AppImage')
    drone_mapping_share = get_package_share_directory('drone_mapping')
    models_path = os.path.join(drone_mapping_share, 'models', 'entities')
    agents_path = os.path.join(drone_mapping_share, 'models', 'agents')

    # Env Vars
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{models_path}:{agents_path}"
    )

    # --- DEFINITIONS ---

    # 1. QGroundControl
    qgc_launch = ExecuteProcess(
        cmd=[qgc_path],
        output='screen'
    )

    # 2. PX4 SITL
    px4_env = {'PX4_GZ_MODEL_POSE': '0,0,0.3,0,0,0'}
    if HEADLESS_MODE:
        px4_env['HEADLESS'] = '1'
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth'],
        cwd=px4_dir,
        output='screen',
        additional_env=px4_env
    )

    mav_config = os.path.join(drone_mapping_share, 'config', 'mavros_config.yaml')

    # 3. MAVROS
    mavros_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'mavros', 'px4.launch',
            f'fcu_url:=udp://:14540@127.0.0.1:14557',
            f'config_yaml:={mav_config}'
        ],
        output='screen'
    )

    # 4b. Spawn Terrain (uncomment to enable)
    spawn_terrain = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'terrain',
            '-x', '5.0',
            '-y', '-5.0',
            '-z', '-0.8',
            '-file', os.path.join(models_path, 'terrain', 'model.sdf')
        ],
        output='screen'
    )
    delayed_terrain = TimerAction(period=8.0, actions=[spawn_terrain])

    # 4. Spawn Rocks (NUM_ROCKS controls how many are active)
    spawn_actions = []
    for cfg in ROCK_CONFIGS[:NUM_ROCKS]:
        spawn_actions.append(
            TimerAction(period=8.0, actions=[Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', cfg['name'],
                    '-x', str(cfg['x']),
                    '-y', str(cfg['y']),
                    '-z', str(cfg['z']),
                    '-R', str(cfg['R']),
                    '-P', str(cfg['P']),
                    '-Y', str(cfg['Y']),
                    '-file', os.path.join(models_path, cfg['model'], 'model.sdf'),
                ],
                output='screen',
            )])
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
    mavros_control_node = Node(
        package='drone_mapping',
        executable='mavros_control',
        output='screen'
    )

    # 7. Bridge ROS-Gazebo Topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/x500_depth_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/depth_camera', '/camera/depth'),
            ('/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image', '/camera/rgb'),
            ('/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info', '/camera/camera_info'),
            ('/model/x500_depth_0/odometry', '/ground_truth/odom')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. Synced Broadcaster (TF + Image Forwarding)
    synced_broadcaster = Node(
        package='drone_mapping',
        executable='synced_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 9. Cluster Tracker
    rock_positions_json = json.dumps([
        {'name': c['name'], 'x': c['x'], 'y': c['y'], 'z': c['z']}
        for c in ROCK_CONFIGS[:NUM_ROCKS]
    ])
    cluster_tracker_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Cluster Tracker', '--', 'bash', '-c',
            f"ROCK_POSITIONS_JSON='{rock_positions_json}'"
            f' ros2 run drone_mapping cluster_tracker --ros-args'
            f' -p num_rocks:={NUM_ROCKS}'
            f' -p trial_id:={TRIAL_ID}'
            '; exec bash'
        ],
        output='screen'
    )

    # 9b. Run Controller (inline — kills everything after LC 6)
    run_controller_node = Node(
        package='drone_mapping',
        executable='run_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 10. RTAB-Map
    rtabmap = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=RTAB-Map', '--', 'bash', '-c',
            'ros2 launch drone_mapping rtabmap.launch.py; exec bash'
        ],
        output='screen'
    )

    # --- SEQUENCE ---
    # T=0s:  QGC
    # T=5s:  PX4
    # T=8s:  Spawn rocks (all at same time)
    # T=12s: MAVROS
    # T=20s: Traversability Node
    # T=24s: Mavros Control Node

    delayed_px4            = TimerAction(period=5.0,  actions=[px4_sitl])
    delayed_mavros         = TimerAction(period=12.0, actions=[mavros_launch])
    delayed_traversability = TimerAction(period=20.0, actions=[traversability_node])
    delayed_control        = TimerAction(period=24.0, actions=[mavros_control_node])
    delayed_rtabmap        = TimerAction(period=20.0, actions=[rtabmap])
    delayed_cluster        = TimerAction(period=22.0, actions=[cluster_tracker_node])
    delayed_controller     = TimerAction(period=25.0, actions=[run_controller_node])

    return LaunchDescription([
        set_gz_resource_path,
        qgc_launch,
        delayed_px4,
        delayed_mavros,
        *spawn_actions,
        delayed_terrain,
        bridge,
        synced_broadcaster,
        delayed_traversability,
        delayed_cluster,
        delayed_control,
        delayed_rtabmap,
        delayed_controller,
    ])
