# thesis_ws

**Autonomous Rock Detection Pipeline — Active Gaussian Splatting for Planetary Reconstruction**
*(IEEE CASE 2026)*

A ROS2 (Humble) workspace that flies a drone over a simulated planetary surface, maps it with RTABMAP, and detects rocks from the accumulated point cloud using a 3-phase geometric pipeline. Detected rock point clouds feed directly into the compression and NBV planning stages of the thesis.

---

## Overview

```
PX4 SITL + Gazebo Harmonic
        │  drone flies lawnmower survey pattern
        ▼
RTABMAP (Visual SLAM)
        │  /rtabmap/cloud_map  (accumulated point cloud)
        │  /rtabmap/info       (loop closure events)
        ▼
Phase 1 — RANSAC Ground Removal        (every frame)
        │  → non-ground points + ground normal
        ▼
Phase 2 — DBSCAN Clustering + Centroid Tracking  (every frame)
        │  → TrackedCluster objects with stability history
        ▼
Phase 3 — Geometric Stability Tracking (loop-closure gated)
        │  → PCA, eigenvector comparison, planarity filter
        ▼
Saved PLY files per rock cluster
        │
        ▼
[Compression + NBV Planning — thesis contributions, separate pipeline]
```

---

## Repository Structure

```
thesis_ws/
└── src/
    └── drone_mapping/
        ├── drone_mapping/          # ROS2 nodes
        │   ├── cluster_tracker.py  # ★ Main node — orchestrates all 3 phases
        │   ├── run_controller.py   # Flight control + auto-shutdown at N loop closures
        │   ├── mavros_control.py   # MAVROS waypoint + arming interface
        │   ├── synced_broadcaster.py # TF publisher with interpolated timestamps
        │   └── traversability.py   # Lawnmower trajectory execution
        ├── drone_utils/            # Pipeline library (no ROS dependency)
        │   ├── phase1_ground_removal.py  # RANSAC ground plane removal
        │   ├── phase2_clustering.py      # DBSCAN + centroid tracking
        │   ├── phase3_geometric.py       # PCA, eigenvector stability, planarity filter
        │   ├── metrics_logger.py         # CSV logging for cluster metrics
        │   ├── trajectory_gui.py         # GUI for waypoint editing
        │   └── plot_centroid_stability.py
        ├── launch/
        │   ├── drone_mapping.launch.py   # ★ Main launch file (full pipeline)
        │   └── rtabmap.launch.py         # RTABMAP SLAM config
        ├── models/
        │   ├── agents/x500_depth_odom/   # Drone SDF (x500 + depth camera)
        │   └── entities/                 # Rock models (rock–rock10) + terrain
        ├── tools/
        │   ├── plot_results.py           # Detection rate plots
        │   ├── plot_pipeline_funnel.py   # Pipeline funnel chart
        │   └── rock_spawner_gui.py       # GUI for manual rock placement
        └── config/
            ├── mavros_config.yaml
            └── trajectory_config.json
```

---

## Dependencies

- ROS2 Humble
- Gazebo Harmonic
- PX4 Autopilot (SITL) — `make px4_sitl gz_x500_depth`
- MAVROS (`ros2 launch mavros px4.launch`)
- RTABMAP ROS2 (`rtabmap_ros`)
- `ros_gz_bridge`
- Python: `numpy`, `scipy`, `scikit-learn`, `open3d`

---

## Quick Start

### 1. Build

```bash
cd thesis_ws
colcon build --packages-select drone_mapping
source install/setup.bash
```

### 2. Configure the experiment

Edit the top of `src/drone_mapping/launch/drone_mapping.launch.py`:

```python
NUM_ROCKS     = 5     # 1–10: how many rocks to spawn in the scene
TRIAL_ID      = 1     # trial index for output file naming
HEADLESS_MODE = True  # False to show Gazebo GUI
```

### 3. Launch the full pipeline

```bash
ros2 launch drone_mapping drone_mapping.launch.py
```

This starts everything in sequence:

| Time | Process |
|------|---------|
| T=0s | QGroundControl |
| T=5s | PX4 SITL |
| T=8s | Spawn rocks + terrain in Gazebo |
| T=12s | MAVROS |
| T=20s | Traversability node + RTABMAP |
| T=22s | Cluster Tracker node |
| T=24s | MAVROS Control node |
| T=25s | Run Controller node |

The drone flies a lawnmower survey pattern and shuts down automatically after 6 loop closures (configurable in `run_controller.py`).

### 4. Launch RTABMAP separately (optional)

```bash
ros2 launch drone_mapping rtabmap.launch.py
```

---

## The 3-Phase Rock Detection Pipeline

All tunable parameters are in the constants block at the top of `cluster_tracker.py`.

### Phase 1 — RANSAC Ground Removal
Runs on every `/rtabmap/cloud_map` frame. Fits a plane to the accumulated point cloud, strips ground points, and caches the plane normal for Phase 3's planarity filter.

| Parameter | Default | Description |
|---|---|---|
| `RANSAC_DIST_THRESHOLD` | `0.05` m | Max distance from plane to count as ground |
| `RANSAC_MAX_TRIALS` | `1000` | RANSAC iterations |
| `PLANE_REUSE_THRESHOLD` | `0.80` | Reuse cached plane if ≥80% of points still fit |

### Phase 2 — DBSCAN Clustering + Centroid Tracking
Runs every frame on the non-ground points. Clusters with DBSCAN, then matches clusters across frames by centroid proximity to maintain stable `TrackedCluster` IDs.

| Parameter | Default | Description |
|---|---|---|
| `DBSCAN_EPS` | `0.1` m | Neighbourhood radius |
| `DBSCAN_MIN_SAMPLES` | `30` | Min points to form a core point |
| `MIN_CLUSTER_SIZE` | `500` | Drop clusters smaller than this |
| `MAX_MATCH_DISTANCE` | `2.0` m | Max centroid shift for ID match |
| `CENTROID_WINDOW` | `5` | Frames for centroid consistency check |
| `CENTROID_CONSISTENCY_THR` | `1.5` m | Max deviation from mean centroid to be active |

### Phase 3 — Geometric Stability (Loop-Closure Gated)
Activates on the first RTABMAP loop closure. Processes **one loop closure per `/rtabmap/cloud_map` update** — this ensures each snapshot sees a genuinely different map state. For each active cluster: computes PCA, compares eigenvectors to the previous snapshot, and checks BBOX stability.

After `PHASE3_LC_WINDOW` loop closures, applies the planarity filter and saves stable clusters as PLY files.

| Parameter | Default | Description |
|---|---|---|
| `PHASE3_LC_WINDOW` | `6` | Loop closures to track before saving |
| `CONVERGENCE_WINDOW` | `2` | Min passing LC pairs required for convergence |
| `PCA_CROSS_THR` | `0.342` (sin 20°) | Max eigenvector deviation between LCs |
| `BBOX_DIM_THR` | `2.0` m | Max BBOX dimension change between LCs |
| `PLANARITY_THR` | `0.174` (sin 10°) | Max angle between smallest PCA axis and ground normal |
| `PLANARITY_RATIO_THR` | `1e-3` | Max `eig2/eig0` — filters truly flat clusters |

---

## Key Nodes

| Node | Executable | Description |
|---|---|---|
| `cluster_tracker` | `cluster_tracker` | Orchestrates Phases 1–3; saves PLY files |
| `run_controller` | `run_controller` | Flies lawnmower, shuts down at N loop closures |
| `mavros_control` | `mavros_control` | Arms drone, sets OFFBOARD mode, sends waypoints |
| `synced_broadcaster` | `synced_broadcaster` | Publishes TF `map→base_link` with interpolated timestamps |
| `traversability` | `traversability` | Generates and executes lawnmower waypoints |

---

## Output Files

Each run saves to `test_runs/RUN_ROCKS_<N>/trial_<ID>/`:

| File | Description |
|---|---|
| `clusters/Cluster_N.ply` | Saved rock point cloud (one per detected rock) |
| `cluster_metrics.csv` | Per-LC stability metrics for each cluster |
| `centroid_log.csv` | Centroid position history across frames |
| `phase_stats.json` | Summary: rocks spawned, detected, false positives |
| `run_metadata.json` | Trial config (NUM_ROCKS, TRIAL_ID, timestamps) |

---

## Experimental Results

30 trials across NUM_ROCKS 1–10, **~92% overall detection rate**.

| NUM_ROCKS | Trial 1 | Trial 2 | Trial 3 | Rate |
|---|---|---|---|---|
| 1 | 1/1 | 1/1 | 1/1 | 100% |
| 2 | 2/2 | 2/2 | 2/2 | 100% |
| 3 | 3/3 | 3/3 | 3/3 | 100% |
| 4 | 4/4 | 4/4 | 4/4 | 100% |
| 5 | 5/5 | 4/5 | 5/5 | 93% |
| 6 | 5/6 | 5/6 | 4/6 | 78% |
| 7 | 6/7 | 6/7 | 7/7 | 90% |
| 8 | 7/8 | 8/8 | 8/8 | 96% |
| 9 | 8/9 | 8/9 | 9/9 | 93% |
| 10 | 7/10 | 10/10 | 10/10 | 90% |

Main failure mode: RTABMAP loop closure corrections merging a rock cluster with surrounding terrain → BBOX explosion → convergence failure (~8% false negatives).

---

## QoS Note

RTABMAP publishes `/rtabmap/cloud_map` and `/rtabmap/info` with **Best Effort** QoS. Both `cluster_tracker` and `run_controller` subscribe with explicit `BEST_EFFORT` QoS — using the default Reliable QoS causes silent message drops and the pipeline receives nothing.
