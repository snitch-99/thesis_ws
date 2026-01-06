# Selective Mesh Refinement for Robotic Mapping

This repository contains the work for the thesis on "Selective Mesh Refinement".

## Overview
The goal of this project is to develop a framework for updating a coarse prior mesh model using high-resolution local point cloud data collected by a robotic platform (e.g., a drone). The core idea is to transmit only the significant geometric updates (discrepancies) to a base station, respecting bandwidth constraints.

## Structure
- **Documents/**: Contains the thesis proposal and related documentation.
- **src/**: Source code for the robotic implementation (ROS 2 packages).
    - `drone_mapping`: Package for drone-based point cloud collection and control.
        - `models/entities`: Contains simulation objects (e.g., rocks).
        - `models/agents`: Contains drone configurations.

---

## Drone Mapping System

The `drone_mapping` package provides the core simulation and data collection pipeline.

### Features
- **Simulation**: Gazebo integration with PX4 SITL (Software In The Loop).
- **Sensors**: Depth Camera (OakD-Lite model) integration.
- **Mapping**: Real-time Point Cloud generation from depth images.
- **Navigation**: Basic traversability analysis and trajectory generation.

### Dependencies
- ROS 2 Humble (or newer)
- Gazebo Garden/Harmonic
- `ros_gz_bridge`
- `depth_image_proc` (for point cloud generation)
- `mavros` & `mavros_extras`
- PX4 Autopilot (for SITL)

### Usage

#### 1. Build the Package
```bash
colcon build --packages-select drone_mapping
source install/setup.bash
```

#### 2. Launch Simulation
This launch file starts Gazebo, the PX4 SITL bridge, MAVROS, and the Point Cloud processing pipeline.
```bash
ros2 launch drone_mapping simulation.launch.py
```

#### 3. Visualization (RViz)
- **Fixed Frame**: `map`
- **Point Cloud Topic**: `/camera/points`
- **Image Topic**: `/camera/image_raw`

### Configuration Notes

#### TF Tree Configuration
The TF tree is managed by `synced_broadcaster.py` to ensure high-fidelity timestamp synchronization:
- **`map` -> `base_link`**: 
    - Published by `synced_broadcaster` by interpolating `mavros/local_position/odom` to match the exact timestamp of depth images.
- **`base_link` -> `camera_link`**:
    - Physical mount frame (Identity rotation, Translation match SDF).
- **`camera_link` -> `camera_link_optical`**:
    - Standard optical rotation (Z-Forward, X-Right, Y-Down).
    - All visual data (Depth, Point Clouds) is expressed in this frame.

#### Point Cloud Generation
We use `depth_image_proc` to generate 3D point clouds.
- **Synchronization**: `synced_broadcaster` ensures `camera_info` and `depth_image` have identical timestamps and correct Optical Frame IDs.
- **Topics**: 
    - Input: `/camera/depth_synced`, `/camera/camera_info_synced`
    - Output: `/camera/points` (in `camera_link_optical` frame)

#### Trajectory Generation
The `traversability` node implements a circular orbit strategy:
- **Logic**: Generates waypoints around a target center using `drone_utils/trajectory_generator.py`.
- **Behavior**: The drone orbits the target while continuously facing the center.
