# Thesis Project Context — Kanav Prashar
## Active Gaussian Splatting for Autonomous Rock Reconstruction (IEEE CASE 2026)

---

## What We Are Building

An autonomous drone pipeline that:
1. **Flies** a lawnmower survey pattern over a planetary surface (PX4 SITL + Gazebo Harmonic)
2. **Maps** the environment in real-time using RTABMAP (visual SLAM + loop closure)
3. **Detects** rocks from the accumulated point cloud using a 3-phase pipeline
4. **Compresses** detected rock point clouds using Superquadrics + DEM (the main thesis contribution)
5. **Plans** next-best-views (NBV) using D-optimal Fisher information (the other main contribution)

The detection pipeline (Phases 1–3) is **plumbing** — it needs to work well enough to feed clean rock point clouds into the compression stage. It is NOT the thesis contribution.

---

## Stack

- **Simulator**: Gazebo Harmonic, PX4 SITL
- **SLAM**: RTABMAP (`qos: '1'` = Best Effort on all published topics)
- **ROS2**: Humble
- **Python env**: `/home/kanav/NBV/gaussian-splatting/venv/bin/python3`
- **Key packages**: `drone_mapping` (in `thesis_ws/src/drone_mapping`)

---

## The 3-Phase Rock Detection Pipeline

### Phase 1 — RANSAC Ground Removal
- Runs on every `/rtabmap/cloud_map` frame
- Fits a plane to the accumulated point cloud, removes ground points
- Caches the plane normal (used by Phase 3 planarity filter)
- Output: non-ground points + ground normal vector

### Phase 2 — DBSCAN Clustering + Centroid Tracking
- DBSCAN on the non-ground points
- Tracks clusters across frames by centroid proximity (Hungarian-style nearest-match)
- Maintains `TrackedCluster` objects with `stability_history`, `lc_matched_count`, `is_compression_candidate`
- After Phase 3 activates: locks new cluster creation (`lock_new_clusters=True`)
- Key parameters: `DBSCAN_EPS=0.1`, `DBSCAN_MIN_SAMPLES=30`, `MIN_CLUSTER_SIZE=500`, `MAX_MATCH_DISTANCE=2.0`, `MAX_MISSING_FRAMES=20`, `CENTROID_WINDOW=5`, `CENTROID_CONSISTENCY_THR=1.0`

### Phase 3 — Geometric Stability Tracking (Loop Closure Gated)
- Activates on first loop closure from RTABMAP
- Runs once per `/rtabmap/cloud_map` update if a pending unprocessed LC exists (**one LC per cloud_map** — key design decision)
- For each active cluster: computes PCA, BBOX, octant occupancy, compares eigenvectors to previous snapshot
- After `PHASE3_LC_WINDOW=5` loop closures: applies planarity filter, sets confidence scores, saves PLY files
- Key parameters: `PHASE3_LC_WINDOW=5`, `CONVERGENCE_WINDOW=2`, `PCA_CROSS_THR=0.3420` (sin 20°), `BBOX_DIM_THR=0.25`, `PLANARITY_THR=0.1736` (sin 10°), `PLANARITY_RATIO_THR=1e-3`

---

## Key Files

| File | Role |
|------|------|
| `drone_mapping/cluster_tracker.py` | Main ROS2 node orchestrating all 3 phases |
| `drone_mapping/run_controller.py` | Controls flight + triggers shutdown at 100 LCs |
| `drone_utils/phase1_ground_removal.py` | RANSAC ground removal |
| `drone_utils/phase2_clustering.py` | DBSCAN + centroid tracking |
| `drone_utils/phase3_geometric.py` | PCA, eigenvector comparison, planarity filter, convergence |
| `drone_utils/metrics_logger.py` | Writes `cluster_metrics.csv` and `centroid_log.csv` |
| `launch/rtabmap.launch.py` | RTABMAP config — `qos: '1'` = Best Effort on all topics |

---

## Critical Fixes Made This Session

### 1. QoS Mismatch (biggest fix)
- RTABMAP publishes `/rtabmap/cloud_map` and `/rtabmap/info` with **Best Effort** QoS (`qos: '1'`)
- Both `cluster_tracker.py` and `run_controller.py` were subscribing with default **Reliable** QoS → silently dropped all messages
- Fixed: explicit `QoSProfile(reliability=BEST_EFFORT, durability=VOLATILE, history=KEEP_ALL)` on both subscriptions in both nodes

### 2. LC Drain Loop: `while` → `if`
- Original code drained ALL pending loop closures in a single `cloud_callback` using a `while` loop
- Problem: if 10 LCs fire before any cloud_map arrives, all 10 Phase 3 iterations see identical geometry → trivially matched → meaningless
- Fixed: changed `while` to `if` → process **one LC per cloud_map update** so each snapshot sees a genuinely different accumulated map

### 3. Logging Bug: `self.loop_closure_count` vs `lc_processed_count`
- `_log_phase3_metrics()` compared `latest.loop_closure_count != self.loop_closure_count`
- With the drain fix, `lc_processed_count` (1..10) never equals `loop_closure_count` (total LCs seen, could be 15+) → nothing logged, CSV empty, PLYs still saved
- Fixed: `_log_phase3_metrics(self.lc_processed_count)` passes the current processed count

### 4. `pending_phase3_compute` bool → `lc_processed_count` int
- Old bool flag was set True multiple times but only cleared once → Phase 3 ran once even if 4 LCs had fired
- Fixed: replaced with integer counter + drain loop (now `if` loop)

### 5. Convergence: consecutive → total passing pairs
- Old `check_convergence`: required last `window` consecutive pairs to ALL pass → one noisy LC in a row failed everything
- Fixed: count total passing pairs across ALL history, require `>= CONVERGENCE_WINDOW` — one bad LC doesn't disqualify

---

## Design Decisions & Rationale

### Why one-LC-per-cloud_map?
RTABMAP loop closure corrections shift global map positions. If multiple LCs fire before any cloud_map update, all Phase 3 iterations see identical geometry. Processing one per cloud_map update ensures each snapshot reflects a genuinely different map state, making eigenvector comparisons meaningful.

### Why keep multi-LC convergence instead of single-shot?
The confidence score (lc_matched_count / PHASE3_LC_WINDOW) discriminates spurious clusters (noise, terrain edges, merged blobs) from real rocks. Phase 2 handles temporal stability; Phase 3 handles geometric self-consistency. The planarity filter alone lets through non-rock 3D clusters.

### Why NOT fix centroid tracking overwriting?
When RTABMAP fires a loop closure, all map points shift. DBSCAN re-clusters and can merge/split clusters. This causes false negatives (real rocks get merged into a large blob and fail convergence). Fixing this is a research problem in its own right and NOT the thesis contribution. 92% overall detection is acceptable.

### Planarity filter
Two conditions BOTH required (so wide-but-tall rocks are kept):
1. `evec2` (smallest PCA axis) aligned with ground normal: `|evec2 × ground_normal| < sin(10°)`
2. Negligible thickness: `eig2 / eig0 < 1e-3`

---

## Known Remaining Issues

### False negatives (~8% of rocks missed)
- **Cluster merging events**: loop closure correction merges a rock's cluster with surrounding points → bbox explodes → convergence fails
- **Late corruption**: rock looks good at LC=1–3, then gets merged into flat blob at LC=4–5 → planarity filter discards it
- **Root cause**: RTABMAP map corrections between LCs are not predictable/stable enough

### Planarity bypass (false positives)
- Flat cluster goes inactive before the final LC → `phase3.update()` never runs on it at LC=5 → planarity never fires → saved as PLY
- A re-check at save time would fix this but was not prioritized (thesis focus is compression + NBV)

### Eigenvector axis swapping
- PCA sorts by descending eigenvalue. If `eig0 ≈ eig1` (roughly symmetric rock), axes can swap rank between LCs → false cross product mismatch
- Raised `PCA_CROSS_THR` to sin(20°) = 0.3420 to mitigate
- Could be fixed by matching eigenvectors by similarity rather than by index — not prioritized

---

## Experimental Results (30 Trials, NUM_ROCKS 1–10)

| NUM_ROCKS | T1 | T2 | T3 | Rate |
|-----------|----|----|-----|------|
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

**Overall: ~92% detection rate across 30 trials**

---

## What Comes Next

### 1. Visualization / Graphs (for thesis figures)
- Detection rate vs NUM_ROCKS (bar chart with per-trial overlay)
- Pipeline funnel chart (DBSCAN → size filter → Phase 2 active → Phase 3 → stable → PLY)
- Planarity discriminator scatter (eig2 vs eig0, rocks vs dropped)
- Confidence score progression over LCs for a representative run
- Per-trial detection heatmap (NUM_ROCKS × trial)

### 2. Compression Pipeline
- Input: saved PLY files from Phase 3
- Method: Superquadric fitting + DEM (Digital Elevation Map) — ~80kB fixed payload
- This is the **main thesis contribution**

### 3. NBV Planning
- D-optimal Fisher information criterion for next-best-view selection
- Gaussian positions frozen (Poisson mesh prior)
- This is the **other main thesis contribution**

---

## Run Data Location
- Test runs: `/home/kanav/workspaces/thesis_ws/test_runs/RUN_ROCKS_<N>/`
- Each trial: `run_metadata.json`, `cluster_metrics.csv`, `centroid_log.csv`, `phase_stats.json`, `clusters/*.ply`
