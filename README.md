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
The TF tree is constructed as follows:
- **`map` -> `base_link`**: 
    - Ideally provided by MAVROS (`global_position` plugin).
    - *Current Fallback*: A static debug publisher is included in `simulation.launch.py` to ensure visualization if MAVROS fails to lock GPS in simulation.
- **`base_link` -> `camera_link`**:
    - Provided by a `static_transform_publisher` in the launch file.
    - Translation: `0.12 0.03 0.242` (Matches SDF)
    - Rotation: Optical frame standard.

#### Point Cloud Generation
We use `depth_image_proc/point_cloud_xyz` composable node to convert the raw depth image from Gazebo to a 3D Point Cloud.
