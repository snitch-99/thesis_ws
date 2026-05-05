"""
Cluster Tracker Node
--------------------
ROS2 node that orchestrates the 3-phase rock detection pipeline:

  Phase 1 — RANSAC ground removal        (every frame)
  Phase 2 — DBSCAN clustering + tracking (every frame)
  Phase 3 — Geometric stability tracking (loop closure gated)

All tunable parameters are in the constants block below.
"""

import json
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

import drone_utils.phase1_ground_removal as phase1
import drone_utils.phase2_clustering     as phase2
import drone_utils.phase3_geometric      as phase3
from drone_utils.phase2_clustering import Phase2State
from drone_utils.metrics_logger    import MetricsLogger

# ═══════════════════════════════════════════════════════════════════════════════
# ALL TUNABLE PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

# ── Phase 1: RANSAC Ground Removal ────────────────────────────────────────────
RANSAC_DIST_THRESHOLD    = 0.05      # Max distance (m) from plane to count as ground
RANSAC_MIN_SAMPLES       = 3        # Min points to fit a plane
RANSAC_MAX_TRIALS        = 1000     # RANSAC iterations
PLANE_REUSE_THRESHOLD    = 0.80     # Reuse cached plane if >=80% points still fit

# ── Phase 2: DBSCAN + Centroid Tracking ───────────────────────────────────────
DBSCAN_EPS               = 0.1      # Neighborhood radius (m)
DBSCAN_MIN_SAMPLES       = 30       # Min points to form a core point
MIN_CLUSTER_SIZE         = 500      # Drop clusters with fewer points than this
MAX_MATCH_DISTANCE       = 2.0      # Max centroid distance (m) for ID match
MAX_MISSING_FRAMES       = 1000      # Drop cluster after unseen this many frames
CENTROID_WINDOW          = 5        # Frames for centroid consistency check
CENTROID_CONSISTENCY_THR = 1.5      # Max deviation (m) from mean centroid to be active

# ── Phase 3: Geometric Tracking (loop closure gated) ──────────────────────────
PHASE3_LC_WINDOW         = 6      # Number of loop closures to track
MIN_POINTS_FOR_PCA       = 50       # Skip Phase 3 for clusters with fewer points
CONVERGENCE_WINDOW       = 2        # Consecutive LC pairs that must all pass
PCA_CROSS_THR            = 0.3420   # sin(20°) — max eigenvector deviation between LCs
BBOX_DIM_THR             = 2.0    # Max BBOX dimension change (m) between LCs
OCTANT_THR               = 1        # Max octant occupancy change between LCs
PLANARITY_THR            = 0.1736   # sin(10°) — max angle between evec2 and ground normal
PLANARITY_RATIO_THR      = 1e-3     # max eig2/eig0 ratio — filters truly flat clusters only

# ═══════════════════════════════════════════════════════════════════════════════

# Cluster colors (RGB 0-1) cycled by cluster ID for visualization
CLUSTER_COLORS = [
    (1.0, 0.2, 0.2),   # red
    (0.2, 0.8, 0.2),   # green
    (0.2, 0.4, 1.0),   # blue
    (1.0, 0.8, 0.1),   # yellow
    (0.9, 0.3, 0.9),   # purple
    (0.1, 0.9, 0.9),   # cyan
    (1.0, 0.5, 0.1),   # orange
    (0.5, 1.0, 0.5),   # light green
]

# Param dicts passed to each phase module
_P1 = dict(
    ransac_dist_threshold = RANSAC_DIST_THRESHOLD,
    ransac_min_samples    = RANSAC_MIN_SAMPLES,
    ransac_max_trials     = RANSAC_MAX_TRIALS,
    plane_reuse_threshold = PLANE_REUSE_THRESHOLD,
)
_P2 = dict(
    dbscan_eps               = DBSCAN_EPS,
    dbscan_min_samples       = DBSCAN_MIN_SAMPLES,
    min_cluster_size         = MIN_CLUSTER_SIZE,
    max_match_distance       = MAX_MATCH_DISTANCE,
    max_missing_frames       = MAX_MISSING_FRAMES,
    centroid_window          = CENTROID_WINDOW,
    centroid_consistency_thr = CENTROID_CONSISTENCY_THR,
)
_P3 = dict(
    phase3_lc_window    = PHASE3_LC_WINDOW,
    min_points_for_pca  = MIN_POINTS_FOR_PCA,
    convergence_window  = CONVERGENCE_WINDOW,
    pca_cross_thr       = PCA_CROSS_THR,
    bbox_dim_thr        = BBOX_DIM_THR,
    octant_thr          = OCTANT_THR,
    planarity_thr       = PLANARITY_THR,
    planarity_ratio_thr = PLANARITY_RATIO_THR,
)


# ── PointCloud2 helpers ───────────────────────────────────────────────────────

def _unpack_pointcloud2(msg: PointCloud2):
    offsets = {f.name: f.offset for f in msg.fields if f.name in ('x', 'y', 'z', 'rgb')}
    if not all(k in offsets for k in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32), None
    step = msg.point_step
    raw  = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.width * msg.height, step)
    xyz  = np.stack([
        np.frombuffer(raw[:, offsets[k]:offsets[k]+4].tobytes(), dtype=np.float32)
        for k in ('x', 'y', 'z')
    ], axis=1)
    valid = np.isfinite(xyz).all(axis=1)
    rgb   = None
    if 'rgb' in offsets:
        rgb = np.frombuffer(
            raw[:, offsets['rgb']:offsets['rgb']+4].tobytes(), dtype=np.float32
        )[valid]
    return xyz[valid], rgb


def _make_pointcloud2(points: np.ndarray, frame_id: str, stamp, rgb: np.ndarray = None) -> PointCloud2:
    msg            = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp    = stamp
    msg.height     = 1
    msg.width      = len(points)
    msg.is_dense   = True
    msg.is_bigendian = False
    pts_f32 = points.astype(np.float32)
    if rgb is not None and len(rgb) == len(points):
        msg.point_step = 16
        msg.row_step   = 16 * msg.width
        msg.fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = np.column_stack([pts_f32, rgb.astype(np.float32)]).tobytes()
    else:
        msg.point_step = 12
        msg.row_step   = 12 * msg.width
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts_f32.tobytes()
    return msg


def _pack_rgb(r: float, g: float, b: float) -> float:
    return np.frombuffer(
        np.array([int(b*255), int(g*255), int(r*255), 0], dtype=np.uint8).tobytes(),
        dtype=np.float32
    )[0]


def _save_ply(points: np.ndarray, path: str) -> None:
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.astype(np.float32).tobytes())


def _make_bbox_marker(cluster, stamp, ns: str) -> Marker:
    pts    = cluster.points
    color  = CLUSTER_COLORS[cluster.cluster_id % len(CLUSTER_COLORS)]
    center = (pts.min(axis=0) + pts.max(axis=0)) / 2.0
    dims   = pts.max(axis=0) - pts.min(axis=0)
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp    = stamp
    m.ns, m.id        = ns, cluster.cluster_id
    m.type            = Marker.CUBE
    m.action          = Marker.ADD
    m.pose.position.x = float(center[0])
    m.pose.position.y = float(center[1])
    m.pose.position.z = float(center[2])
    m.pose.orientation.w = 1.0
    m.scale.x = float(max(dims[0], 0.01))
    m.scale.y = float(max(dims[1], 0.01))
    m.scale.z = float(max(dims[2], 0.01))
    m.color   = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.35)
    m.lifetime = Duration(sec=2)
    return m


def _make_text_marker(cluster, stamp, ns: str, phase3_active: bool) -> Marker:
    color    = CLUSTER_COLORS[cluster.cluster_id % len(CLUSTER_COLORS)]
    bbox_max = cluster.points.max(axis=0)
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp    = stamp
    m.ns, m.id        = ns + '_text', cluster.cluster_id
    m.type            = Marker.TEXT_VIEW_FACING
    m.action          = Marker.ADD
    m.pose.position.x = float(cluster.centroid[0])
    m.pose.position.y = float(cluster.centroid[1])
    m.pose.position.z = float(bbox_max[2]) + 0.3
    m.pose.orientation.w = 1.0
    m.scale.z = 0.4
    m.color   = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
    label = f'ID:{cluster.cluster_id} ({len(cluster.points)}pts)'
    if phase3_active and cluster.stability_history:
        latest = cluster.stability_history[-1]
        label += f' [{latest.octant_occupancy}/8]'
        if cluster.is_compression_candidate:
            label += ' STABLE'
    m.text     = label
    m.lifetime = Duration(sec=2)
    return m


# ── Main Node ─────────────────────────────────────────────────────────────────

class ClusterTrackerNode(Node):

    def __init__(self):
        super().__init__('cluster_tracker')

        # Phase 1 state
        self.plane_cache = []   # mutable cache: [] or [coef (3,)]

        # Phase 2 state
        self.p2_state = Phase2State()

        # Phase 3 state
        self.loop_closure_count        = 0
        self.lc_processed_count        = 0   # how many LCs Phase 3 has already processed
        self.phase3_active             = False
        self.last_lc_id                = 0

        # ── Experiment metadata parameters ────────────────────────────────────
        self.declare_parameter('num_rocks', 0)
        self.declare_parameter('trial_id',  0)

        num_rocks      = self.get_parameter('num_rocks').value
        trial_id       = self.get_parameter('trial_id').value
        rock_positions = json.loads(os.environ.get('ROCK_POSITIONS_JSON', '[]'))

        # Logging
        self.metrics_logger = MetricsLogger(
            num_rocks=num_rocks,
            trial_id=trial_id,
            rock_positions=rock_positions,
        )
        self.cloud_publishers = {}   # int -> Publisher (one per tracked cluster)

        # ── Publishers ────────────────────────────────────────────────────────
        self.ground_removed_pub  = self.create_publisher(PointCloud2, '/cluster_tracker/phase1_op_ground_removed',      10)
        self.tracked_clusters_pub= self.create_publisher(PointCloud2, '/cluster_tracker/phase2_op_tracked_clusters',   10)
        self.count_pub           = self.create_publisher(Int32,       '/cluster_tracker/phase2_op_count',              10)
        self.stable_clusters_pub = self.create_publisher(PointCloud2, '/cluster_tracker/phase3_op_stable_clusters',    10)
        self.marker_pub          = self.create_publisher(MarkerArray, '/cluster_tracker/phase3_op_markers',            10)

        # ── Subscribers ───────────────────────────────────────────────────────
        _best_effort_qos = QoSProfile(
            depth       = 0,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_ALL,
        )
        self.create_subscription(PointCloud2, '/rtabmap/cloud_map', self.cloud_callback, _best_effort_qos)
        try:
            from rtabmap_msgs.msg import Info
            self.create_subscription(Info, '/rtabmap/info', self.info_callback, _best_effort_qos)
            self.get_logger().info('Subscribed to /rtabmap/info for loop closure detection')
        except ImportError:
            self.get_logger().warn('rtabmap_msgs not available — loop closure gating disabled')

        self.get_logger().info(
            f'Cluster Tracker ready | '
            f'Phase 3 activates on first loop closure, tracks for {PHASE3_LC_WINDOW} LCs'
        )
        self.get_logger().info(f'Logging to: {self.metrics_logger.run_dir}')

    # ── Loop closure callback ─────────────────────────────────────────────────

    def info_callback(self, msg):
        if msg.loop_closure_id > 0 and msg.loop_closure_id != self.last_lc_id:
            self.last_lc_id          = msg.loop_closure_id
            self.loop_closure_count += 1
            self.get_logger().info(f'Loop closure #{self.loop_closure_count} (ID={msg.loop_closure_id})')

            if not self.phase3_active:
                self.phase3_active = True
                self.get_logger().info(
                    f'Phase 3 activated — geometric tracking for {PHASE3_LC_WINDOW} loop closures'
                )

            # No flag needed — cloud_callback drains the gap between
            # lc_processed_count and loop_closure_count each frame.

    # ── Main pipeline ─────────────────────────────────────────────────────────

    def cloud_callback(self, msg: PointCloud2):
        # ── Step 1: Deserialize ───────────────────────────────────────────────
        points, _ = _unpack_pointcloud2(msg)
        if len(points) < 10:
            return

        # ── Step 2: Phase 1 — Ground removal ─────────────────────────────────
        non_ground, self.plane_cache = phase1.remove_ground(points, self.plane_cache, _P1)
        if len(non_ground) < DBSCAN_MIN_SAMPLES:
            return
        self._publish_ground_removed(non_ground, msg)

        # ── Step 3: Phase 2 — DBSCAN + centroid tracking ─────────────────────
        self.p2_state = phase2.cluster_and_track(
            non_ground, self.p2_state, _P2,
            lock_new_clusters=self.phase3_active,
        )
        self._publish_tracked_clusters(msg)
        self._log_centroids()

        # ── Step 4: Phase 3 — one LC per cloud_map update ────────────────────
        # Process at most one pending LC per cloud_callback so each snapshot
        # sees a genuinely different point cloud, not the same frame N times.
        # No window cap — Phase 3 keeps evaluating at every LC indefinitely.
        if (self.phase3_active
               and self.lc_processed_count < self.loop_closure_count):
            self.lc_processed_count += 1
            ground_normal = phase1.get_ground_normal(self.plane_cache)
            self.p2_state.tracked = phase3.update(
                self.p2_state.tracked, ground_normal,
                self.lc_processed_count, _P3,
            )
            self._log_phase3_metrics(self.lc_processed_count)
            self._publish_stable_clusters(msg)
            # Save every PHASE3_LC_WINDOW LCs so we get periodic snapshots
            if self.lc_processed_count % PHASE3_LC_WINDOW == 0:
                self._save_cluster_pointclouds()

        # ── Step 5: Visualization ─────────────────────────────────────────────
        self._publish_per_cluster_clouds(msg)
        self._publish_markers(msg.header.stamp)
        self._publish_count()

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_ground_removed(self, non_ground, msg):
        """Publish Phase 1 output — non-ground point cloud."""
        self.ground_removed_pub.publish(
            _make_pointcloud2(non_ground, msg.header.frame_id, msg.header.stamp)
        )

    def _publish_tracked_clusters(self, msg):
        """Publish Phase 2 output — clusters with stable centroids (is_active=True)."""
        all_pts, all_rgb = [], []
        for tc in self.p2_state.tracked.values():
            if not tc.is_active or tc.frames_missing > 0:
                continue
            packed = _pack_rgb(*CLUSTER_COLORS[tc.cluster_id % len(CLUSTER_COLORS)])
            all_pts.append(tc.points)
            all_rgb.append(np.full(len(tc.points), packed, dtype=np.float32))
        if all_pts:
            self.tracked_clusters_pub.publish(
                _make_pointcloud2(
                    np.vstack(all_pts), msg.header.frame_id, msg.header.stamp,
                    np.concatenate(all_rgb),
                )
            )

    def _publish_stable_clusters(self, msg):
        """Publish Phase 3 output — clusters that passed the planarity filter."""
        all_pts, all_rgb = [], []
        for tc in self.p2_state.tracked.values():
            if not tc.is_active or tc.frames_missing > 0:
                continue
            if not tc.is_compression_candidate:
                continue
            packed = _pack_rgb(*CLUSTER_COLORS[tc.cluster_id % len(CLUSTER_COLORS)])
            all_pts.append(tc.points)
            all_rgb.append(np.full(len(tc.points), packed, dtype=np.float32))
        if all_pts:
            self.stable_clusters_pub.publish(
                _make_pointcloud2(
                    np.vstack(all_pts), msg.header.frame_id, msg.header.stamp,
                    np.concatenate(all_rgb),
                )
            )

    def _publish_per_cluster_clouds(self, msg):
        """Publish one PointCloud2 per tracked cluster on /cluster_tracker/cluster_<ID>."""
        active_ids = set(self.p2_state.tracked.keys())
        for stale_id in set(self.cloud_publishers) - active_ids:
            self.destroy_publisher(self.cloud_publishers.pop(stale_id))
        for tc in self.p2_state.tracked.values():
            if not tc.is_compression_candidate:
                continue
            if tc.cluster_id not in self.cloud_publishers:
                topic = f'/cluster_tracker/phase3_op_cluster_{tc.cluster_id}'
                self.cloud_publishers[tc.cluster_id] = self.create_publisher(PointCloud2, topic, 10)
            packed = _pack_rgb(*CLUSTER_COLORS[tc.cluster_id % len(CLUSTER_COLORS)])
            rgb    = np.full(len(tc.points), packed, dtype=np.float32)
            self.cloud_publishers[tc.cluster_id].publish(
                _make_pointcloud2(tc.points, msg.header.frame_id, msg.header.stamp, rgb)
            )

    def _publish_markers(self, stamp):
        markers = []
        for tc in self.p2_state.tracked.values():
            if tc.frames_missing > 0 or not tc.is_compression_candidate:
                continue
            markers.append(_make_bbox_marker(tc, stamp, 'cluster_bbox'))
            markers.append(_make_text_marker(tc, stamp, 'cluster_bbox', self.phase3_active))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _publish_count(self):
        active = sum(1 for tc in self.p2_state.tracked.values() if tc.frames_missing == 0)
        self.count_pub.publish(Int32(data=active))

    # ── Logging helpers ───────────────────────────────────────────────────────

    def _log_centroids(self):
        """Log centroid position + PCA for every present cluster (Phase 2 output)."""
        for tc in self.p2_state.tracked.values():
            if tc.frames_missing == 0:
                eigenvalues, eigenvectors = None, None
                if tc.points is not None and len(tc.points) >= 3:
                    try:
                        centered = tc.points - tc.points.mean(axis=0)
                        cov = np.cov(centered.T)
                        eig_vals, eig_vecs = np.linalg.eigh(cov)
                        # Sort descending (eigh returns ascending)
                        idx = np.argsort(eig_vals)[::-1]
                        eigenvalues  = eig_vals[idx]
                        eigenvectors = eig_vecs[:, idx]
                    except Exception:
                        pass
                self.metrics_logger.log_centroid(
                    tc.cluster_id, self.p2_state.map_update_count,
                    tc.centroid, tc.is_active,
                    eigenvalues=eigenvalues, eigenvectors=eigenvectors,
                )

    def _log_phase3_metrics(self, lc_count: int):
        """Log PCA snapshot for every cluster processed in Phase 3 at the given lc_count."""
        for tc in self.p2_state.tracked.values():
            if not tc.stability_history:
                continue
            latest = tc.stability_history[-1]
            if latest.loop_closure_count != lc_count:
                continue
            self.metrics_logger.log(tc.cluster_id, latest, is_stable=tc.is_compression_candidate,
                                    map_update=self.p2_state.map_update_count)
            self.get_logger().info(
                f'  Cluster {tc.cluster_id}: '
                f'octant={latest.octant_occupancy}/8  '
                f'bbox=[{latest.bbox_dims[0]:.2f},{latest.bbox_dims[1]:.2f},{latest.bbox_dims[2]:.2f}]  '
                f'conf={latest.confidence:.2f}  '
                f'{"STABLE" if tc.is_compression_candidate else ""}'
            )

    # ── Save PLY ──────────────────────────────────────────────────────────────

    def _save_cluster_pointclouds(self):
        clouds_dir = os.path.join(self.metrics_logger.run_dir, 'clusters')
        os.makedirs(clouds_dir, exist_ok=True)
        for tc in self.p2_state.tracked.values():
            if tc.frames_missing > 0 or not tc.is_compression_candidate:
                continue
            path = os.path.join(clouds_dir, f'cluster_{tc.cluster_id}.ply')
            _save_ply(tc.points, path)
            self.get_logger().info(f'Saved cluster {tc.cluster_id} ({len(tc.points)} pts) → {path}')
        self._save_phase_stats()

    def _save_phase_stats(self):
        """Write pipeline funnel counts to phase_stats.json for plotting."""
        import csv as _csv

        # Derive Phase 2 / Phase 3 counts from CSVs + state
        p2_unique_ids   = self.p2_state.next_id           # total unique IDs ever assigned
        p2_active_ids   = set()
        p3_reached_ids  = set()
        p3_stable_ids   = set()
        p3_filtered_ids = set()

        centroid_path = os.path.join(self.metrics_logger.run_dir, 'centroid_log.csv')
        if os.path.exists(centroid_path):
            with open(centroid_path) as f:
                for row in _csv.DictReader(f):
                    if row['is_active'] == '1':
                        p2_active_ids.add(int(row['cluster_id']))

        metrics_path = os.path.join(self.metrics_logger.run_dir, 'cluster_metrics.csv')
        if os.path.exists(metrics_path):
            with open(metrics_path) as f:
                for row in _csv.DictReader(f):
                    cid = int(row['cluster_id'])
                    p3_reached_ids.add(cid)
                    if row['is_stable'] == '1':
                        p3_stable_ids.add(cid)
                    else:
                        p3_filtered_ids.add(cid)

        stats = {
            "phase2_raw_dbscan_detections": self.p2_state.raw_dbscan_count,
            "phase2_size_dropped":          self.p2_state.size_dropped_count,
            "phase2_unique_ids_tracked":    p2_unique_ids,
            "phase2_active_clusters":       len(p2_active_ids),
            "phase3_reached":               len(p3_reached_ids),
            "phase3_planarity_dropped":     len(p3_filtered_ids - p3_stable_ids),
            "phase3_stable_output":         len(p3_stable_ids),
        }

        path = os.path.join(self.metrics_logger.run_dir, 'phase_stats.json')
        with open(path, 'w') as f:
            json.dump(stats, f, indent=2)
        self.get_logger().info(f'Phase stats saved → {path}')

    def destroy_node(self):
        self.metrics_logger.close()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ClusterTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
