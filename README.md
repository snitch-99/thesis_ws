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
- `rtabmap_ros` (for SLAM)
- `cv_bridge`
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

#### 3. Launch RTAB-Map (SLAM)
This launches RTAB-Map for 3D mapping and loop closure detection.
```bash
ros2 launch drone_mapping rtabmap.launch.py
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
The `traversability` node implements multiple trajectory patterns using `drone_utils/trajectory_generator.py`:
- **Patterns**: Circular, Square.
- **Behavior**: The drone traverses the generated waypoints while maintaining a specific heading (facing center for Orbit).

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

## Change Detection & Point Cloud Segregation
We have implemented a robust system to detect changes between a high-resolution scan and a base model.

### Workflow
1.  **Segregation**: 
    - Incoming point clouds are processed to remove the ground plane (using RANSAC).
    - Remaining objects are clustered using Euclidean Clustering (DBSCAN) to separate distinct entities (e.g., rocks, obstacles).
2.  **Descriptor Assignment**:
    - Each cluster is analyzed to extract key properties, forming a unique descriptor:
        - **Centroid**: (x, y, z) position.
        - **Bounding Box**: Axis-aligned geometric extent.
        - **Dimensions**: Width, Height, Depth.
        - **Point Count**: Density/Volume proxy.
        - **Fractal Dimension**: Metric for surface complexity (e.g., ~2.04 for rocks).
        - **PCA Analysis**: Eigenvalues and Eigenvectors to determine object orientation and principal axes.
3.  **Change Detection**:
    - We employ a `compare_clusters` function that checks the Euclidean distance between centroids, dimensions, eigenvalues, and eigenvectors (handling sign ambiguity).
    - **Threshold**: A delta threshold (e.g., 1.5) determines if an object is a match or new.
    - **New Objects**: Clusters with no matching descriptor in the base database are flagged as **New Objects** and saved to `scene_delta.ply`.
    
### Visualization
The `visualize_pcd.py` script now launches three parallel Open3D windows:
1.  **Base Cloud**: The original environment.
2.  **Changed Cloud**: The new scan containing updates.
3.  **New Objects Detected**: A dedicated view isolating only the new or changed entities, highlighted in Red.
Values are visualized with a 30m Grid and Coordinate Frames for reference.

### Implementation
- The logic is encapsulated in `src/point_cloud_processing/src/visualize_pcd.py`.
- **Modular Design**:
    - `cluster_filtering`: Handles ground removal and DBSCAN clustering.
    - `cluster_properties`: Computes and assigns descriptors (centroid, bbox, PCA, Fractal Dim) to each cluster.
    - `compare_clusters`: Logic for differencing and matching objects.

## Point Cloud Compression Pipeline

We have developed a semantic compression system that achieves **16:1 compression** on deviation data using parametric surface representations.

### Architecture

#### 1. **Global Superquadric Fitting** (`super_quadrics_reduction.py`)
- Fits a base Superquadric to the entire point cloud using the EMS (Expectation-Maximization-Switching) algorithm
- **Parameters**: 5 shape params (ax, ay, az, e1, e2) + 3 center + 9 rotation = 17 floats
- **Output**: 
  - `results/output.txt`: Detailed fitting statistics (SQ params, bbox, PCA, inlier/outlier counts)
  - `encoded_data/sq_params.npy`, `sq_center.npy`, `sq_rotation.npy`: Encoded parameters
  - `viz_output/05_Deviation_Points_>10cm.ply`: Points >10cm from SQ surface

#### 2. **Deviation Patch Analysis** (`deviation_reduction.py`)
- **Clustering**: DBSCAN + K-Means to identify distinct deviation patches
- **Ray-Based Filtering**: Constructs ray from Global SQ center through patch centroid, filters points within 30° cone
- **EM Paraboloid Fitting**: Fits paraboloid to each patch using iterative EM algorithm
  - **E-Step**: Classifies points as inliers (<5cm) or outliers
  - **M-Step**: Re-fits paraboloid using only inliers
  - **Convergence**: Typically 5-10 iterations
- **Output**: `viz_output/paraboloid_fitting_results.csv` with per-patch statistics

#### 3. **Paraboloid Fitting** (`geometry_fitting.py`)
- `fit_paraboloid()`: Standard least-squares fitting (z = ax² + by² + cxy + dx + ey + f)
- `fit_paraboloid_em()`: EM-based robust fitting with automatic inlier/outlier classification
- `generate_paraboloid_points()`: Surface reconstruction from parameters

### Compression Results

**Input**: 249,464 deviation points × 3 coordinates = **748,392 floats**

**Compressed Representation**:
- Global Superquadric: 17 floats
- 60 Paraboloid patches: 360 floats (60 × 6 parameters)
- 15,353 Outliers: 46,059 floats (unexplained points)
- **Total**: **46,436 floats**

**Compression Ratio**: **16.12:1** (93.8% reduction)

### Key Features
- **Semantic Compression**: Preserves geometric meaning (base shape + bumps)
- **Parametric Representation**: Editable parameters vs. opaque bitstreams
- **Bounded Error**: 5cm threshold ensures predictable reconstruction quality
- **Automatic Segmentation**: No manual intervention required

### Usage

#### Run Global Superquadric Fitting
```bash
cd src/point_cloud_processing/src/superquadrics
python3 super_quadrics_reduction.py
```

#### Run Deviation Patch Analysis
```bash
cd src/point_cloud_processing/src/deviation
python3 deviation_reduction.py
```

### Future Work
- **Decoder Implementation**: Reconstruct geometry from compressed parameters
- **Validation**: Measure reconstruction error and visual fidelity
- **Optimization**: Reduce decode time for real-time applications
