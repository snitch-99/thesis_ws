# Cluster Tracking Node — Handoff Document
## Project: Autonomous Geometric Compression for Extraterrestrial Exploration (ASU Master's Thesis)

---

## What We Built

A two-phase ROS2 cluster tracking node that segregates a 3D point cloud scene into individual geological entities (rocks) and determines when each cluster is "stable enough" to be passed to the compression pipeline (Superquadric fitting + Deviation Map).

### Pipeline Overview
```
/rtabmap/cloud_map (PointCloud2)
    → RANSAC ground removal (cached plane)
    → DBSCAN clustering
    → Filter small clusters (< 10% of largest)
    → Centroid-based ID matching (with long-term memory)
    → [Phase 2] PCA eigenvector stability + BBOX + octant occupancy tracking
    → Publish per-cluster PointCloud2 + markers + non-ground cloud
    → [After 10th LC] Save cluster PLY files to test_runs/
```

---

## File Structure

```
thesis_ws/src/drone_mapping/
    drone_mapping/
        cluster_tracker.py          # Main ROS2 node (two-phase tracking)
    drone_utils/
        ground_removal.py           # RANSAC ground plane removal (with plane caching)
        clusterer.py                # DBSCAN + small cluster filter
        cluster_id_tracker.py       # Centroid matching + long-term memory + StabilitySnapshot
        stability_metrics.py        # PCA, BBOX, octant occupancy, convergence check
        metrics_logger.py           # CSV logger → test_runs/run_<timestamp>/cluster_metrics.csv

thesis_ws/test_runs/
    run_<timestamp>/
        cluster_metrics.csv         # Per-cluster stability metrics per loop closure
        clusters/
            cluster_0.ply           # Saved point clouds after 10th loop closure
            cluster_1.ply
            ...
```

---

## Two-Phase Design

### Phase 1 — Pre Loop Closure (Centroid Only)
- Tracking starts immediately on node launch
- Only centroid-based matching — map is drifting, PCA/BBOX are unreliable
- Long-term memory: clusters that disappear and reappear near the same centroid get their old ID back (never forgotten)

### Phase 2 — Post Loop Closure (Full Stability Tracking)
- Activates on the **first loop closure** detected from `/rtabmap/info`
- Tracks for **10 loop closures** (`PHASE2_TRACKING_WINDOW = 10`)
- At each loop closure, computes per-cluster:
  - **PCA eigenvectors** — stored as (3,3) matrix, columns = principal axes
  - **BBOX dimensions** — axis-aligned extents [x, y, z]
  - **Octant occupancy** — how many of 8 octants around centroid have points (1-8)
- Convergence check (cluster is a compression candidate) requires last 3 consecutive pairs stable:
  - `||e_i(t) x e_i(t-1)|| < sin(10°) = 0.1736` for all 3 axes
  - BBOX change < 0.25m in any dimension
  - Octant occupancy change < 1
- **Timing fix:** metrics are computed on the next `cloud_callback` AFTER a loop closure, not during `info_callback` — this ensures the corrected map (not the ghost-laden intermediate state) is used

---

## Key Parameters

| Parameter | Value | Location |
|-----------|-------|----------|
| `RANSAC_DIST_THRESHOLD` | 0.25m | cluster_tracker.py |
| `DBSCAN_EPS` | 0.1m | cluster_tracker.py |
| `DBSCAN_MIN_SAMPLES` | 30 | cluster_tracker.py |
| `SMALL_CLUSTER_THRESHOLD` | 10% of largest | cluster_tracker.py |
| `MAX_MATCH_DISTANCE` | 2.0m | cluster_id_tracker.py |
| `MAX_MISSING_FRAMES` | 5 | cluster_id_tracker.py |
| `PHASE2_TRACKING_WINDOW` | 10 loop closures | cluster_tracker.py |
| `PCA_CROSS_PRODUCT_THRESHOLD` | 0.1736 (sin 10°) | stability_metrics.py |
| `BBOX_DIM_THRESHOLD` | 0.25m | stability_metrics.py |
| `OCTANT_THRESHOLD` | 1 | stability_metrics.py |
| `CONVERGENCE_WINDOW` | 3 pairs | stability_metrics.py |
| `PLANE_REUSE_THRESHOLD` | 80% inlier ratio | ground_removal.py |
| `MIN_POINTS_FOR_STABILITY` | 50 pts | stability_metrics.py |

---

## Difficulties Faced and How We Overcame Them

### 1. Merge-Recover Cycles (Biggest Problem)
**Problem:** Every few loop closures, DBSCAN would merge multiple separate rocks into a single mega-cluster (BBOX expanding from ~2m to 10-15m), then recover a few frames later. This made stability metrics meaningless.

**Root Cause:** Two compounding issues:
- RANSAC was recomputing a new ground plane on every frame. Slight variations in the fitted plane shifted the non-ground point boundary, giving DBSCAN a different input each time.
- Loop closures temporarily create "ghost" point clouds — the old (pre-correction) scans overlap with corrected positions, inflating clusters enough for DBSCAN to merge them.

**Fix 1 — RANSAC Plane Caching:** Store the fitted plane coefficients `[a, b, c]`. On each frame, check if the cached plane still fits ≥ 80% of points. If yes, reuse it (skip RANSAC entirely). Only recompute when the ground genuinely changes. This eliminated inter-frame DBSCAN instability entirely.

**Fix 2 — DBSCAN eps reduction:** Reduced `eps` from 0.3m to 0.1m. At 0.3m, rocks that were 0.2-0.3m apart would bleed into each other under ghost point conditions. At 0.1m, only points truly within the same rock cluster together.

**Fix 3 — Deferred stability computation:** Changed metrics to compute on the next cloud_callback after a loop closure (not immediately in info_callback), ensuring ghost points have been cleared from the map.

**Result:** Latest run (`run_20260327_110725`) — all 4 clusters tracked cleanly across all 10 loop closures with zero merge events.

### 2. Cluster ID Loss After Disappearance
**Problem:** When a cluster temporarily disappeared (merged into another or occluded), after MAX_MISSING_FRAMES=5 it was dropped. When it reappeared, it got a new ID, breaking continuity and resetting its stability history.

**Fix:** Added a long-term memory dict (`self.cluster_memory`) that stores every cluster's last known centroid **forever**. Matching priority:
1. Active tracked clusters first
2. Long-term memory (resurrect old ID)
3. New ID only if nothing matches within 2m

### 3. Eigenvector vs Eigenvalue for Stability
**Original idea:** Track eigenvalue changes between snapshots.

**Problem:** Eigenvalues can be similar even if the shape orientation has rotated (e.g., near-symmetric rocks). Also, eigenvalues depend on point count, making them noisy for partially observed clusters.

**Fix:** Track eigenvectors instead, using cross-product magnitude `||e_i(t) x e_i(t-1)||` = sin(angle). Sign ambiguity is handled naturally (v and -v are parallel, cross product → 0). Threshold: sin(10°) = 0.1736.

### 4. PCA Meaningless Before Full Observation
**Key insight (Kanav):** Before loop closure, the centroid is the only metric that meaningfully stabilizes. PCA and BBOX keep changing as the drone sees new sides of the rock. Hence the two-phase design — centroid-only in Phase 1, full metrics only in Phase 2.

### 5. Octant Occupancy for Completeness
**Purpose:** Tracks whether the drone has observed the rock from all directions. 8 octants around the centroid defined by sign(x-cx), sign(y-cy), sign(z-cz). All 8 filled = rock observed from all directions.

**Implementation:** `sign_matrix @ [4, 2, 1]` maps each point to an octant index 0-7. `len(np.unique(indices))` gives count. No loops, fully vectorized.

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cluster_tracker/cloud_{id}` | PointCloud2 | Per-cluster point cloud (dynamic) |
| `/cluster_tracker/markers` | MarkerArray | BBOX + ID text in RViz |
| `/cluster_tracker/count` | Int32 | Number of active clusters |
| `/cluster_tracker/non_ground` | PointCloud2 | Post-RANSAC cloud before clustering |

---

## What's Next

1. **Convergence threshold validation** — The thresholds (sin 10°, 0.25m BBOX) were set based on reasoning. Need to validate from more test runs across different scenarios. The CSV logs are the tool for this.

2. **Noise testing** — Add Gaussian noise to depth sensor in Gazebo SDF and validate stability metrics degrade gracefully. Camera model is `oakd_gimbal/model.sdf` (640x360, 30Hz, fx/fy=465.86).

3. **Lunar terrain testing** — Terrain model at `models/entities/terrain/model.sdf`. Z-scale was changed from 0.0001 → 0.1 to give realistic vertical relief. Need to verify rocks sit correctly on terrain and RANSAC still cleanly separates ground from rocks.

4. **Chapter 3 writeup** — The implementation now matches the thesis description. Sections to finalize:
   - 3.3.2 should describe eigenvector cross-product stability (not eigenvalues)
   - Add octant occupancy description
   - Document the RANSAC plane caching as a practical engineering contribution
   - Report convergence threshold derivation as part of Chapter 5 experiments

5. **Compression pipeline handoff** — Once a cluster is marked `is_compression_candidate = True`, its PLY file is saved. The next step is feeding it into the Superquadric (EMS) fitting node.

---

## Simulation Setup

- **Drone model:** `x500_depth` (includes `oakd_gimbal`)
- **Camera:** OAK-D Lite, 640x360, 30Hz, depth clip 0.2-19.1m
- **RTAB-Map launch:** `rtabmap.launch.py` — `cloud_decimation=1`, `cloud_voxel_size=0.0`, `Grid/CellSize=0.01`
- **ROS2 distro:** Jazzy
- **Simulator:** Gazebo Harmonic
- **Build:** `colcon build --packages-select drone_mapping`
- **Run:** `ros2 run drone_mapping cluster_tracker`
