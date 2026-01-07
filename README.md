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
    - `drone_interfaces`: Package containing custom ROS 2 message definitions (e.g., `SyncedPointCloud`).

---

## System Architecture

### Two-Package Setup
The project uses a modular two-package architecture:
1.  **`drone_interfaces`**: 
    - Purely defines custom message types (e.g., `SyncedPointCloud`).
    - Ensures clean separation of interface definitions from implementation logic.
    - Builds as an `ament_cmake` package to support IDL generation.
2.  **`drone_mapping`**:
    - Contains the core Python logic (`ament_python` build type).
    - Implements nodes for simulation, control, and data processing.
    - Imports messages from `drone_interfaces`.

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

## Challenges & Solutions

### 1. Point Cloud Instability & Drift
**Problem**: Initial point clouds were unstable and drifted significantly in RViz, even when the drone was hovering.
**Root Cause**: The default timestamps from PX4/MAVROS (Odometry) and Gazebo (Camera) were loosely coupled. Depth projection uses the TF tree at the *exact* timestamp of the image. Mismatches caused the projection to use an outdated or future robot pose, resulting in "smearing" or "jittering" of the point cloud.
**Solution**: Implemented `synced_broadcaster.py`:
- **Buffers** high-frequency Odometry data.
- **Interpolates** the robot pose to the **exact nanosecond timestamp** of each incoming depth frame.
- **Publishes** the TF transform `map -> base_link` with that specific timestamp.
- **Result**: Perfectly registered point clouds that remain stable during motion.

### 2. Coordinate Frame Hell (NED vs ENU vs Optical)
**Problem**: The drone would fly correctly but the camera data pointed at the sky or was mirrored.
**Context**: 
- **PX4** uses FRD (Forward-Right-Down) body frame and NED (North-East-Down) world frame.
- **ROS 2** uses FLU (Forward-Left-Up) body frame and ENU (East-North-Up) world frame.
- **Cameras** use Optical frames (Z-Forward, X-Right, Y-Down).
**Solution**: 
- Strict adherence to ROS REP-103 standards.
- Manually defined static transforms in `synced_broadcaster` to bridge the gap:
    - `base_link` (FLU) -> `camera_link` (Physical Mount).
    - `camera_link` -> `camera_link_optical` (Optical Rotation).
- MAVROS handles the FRD <-> FLU conversion automatically, but the camera frames required explicit management.
