import csv
import json
import os
from datetime import datetime

from drone_utils.phase3_geometric import eigenvector_cross as compute_eigenvector_stability

# ── Constants ──
BASE_OUTPUT_DIR = os.path.expanduser("~/workspaces/thesis_ws/test_runs")

CSV_HEADER = [
    "cluster_id",
    "loop_closure_count",
    "map_update",
    "pca_eig_0", "pca_eig_1", "pca_eig_2",
    # Eigenvectors: columns of the (3,3) matrix, each column is one principal axis
    # evec0 = most dominant axis, evec2 = least dominant axis
    "evec0_x", "evec0_y", "evec0_z",
    "evec1_x", "evec1_y", "evec1_z",
    "evec2_x", "evec2_y", "evec2_z",
    "cross_0", "cross_1", "cross_2",
    "bbox_x", "bbox_y", "bbox_z",
    "octant_occupancy",
    "confidence",       # lc_matched_count / PHASE3_LC_WINDOW (set on final LC)
    "is_stable",        # 1 = passed planarity filter, is a compression candidate; 0 = filtered out
]


class MetricsLogger:
    """Logs per-cluster stability snapshots to a timestamped CSV file in test_runs/."""

    def __init__(self, base_dir: str = BASE_OUTPUT_DIR,
                 num_rocks: int = 0, trial_id: int = 0,
                 rock_positions: list = None):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(base_dir, f"run_{timestamp}")
        os.makedirs(run_dir, exist_ok=True)
        self._run_dir = run_dir

        # ── Write run metadata ────────────────────────────────────────────────
        metadata = {
            "num_rocks":      num_rocks,
            "trial_id":       trial_id,
            "timestamp":      timestamp,
            "rock_positions": rock_positions or [],
        }
        with open(os.path.join(run_dir, "run_metadata.json"), "w") as f:
            json.dump(metadata, f, indent=2)
        self._path = os.path.join(run_dir, "cluster_metrics.csv")
        self._file = open(self._path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_HEADER)
        self._file.flush()

        # Cache previous eigenvectors per cluster for cross-product computation
        self._prev_evecs = {}  # dict[int, np.ndarray]

        # Centroid log
        self._centroid_path = os.path.join(run_dir, "centroid_log.csv")
        self._centroid_file = open(self._centroid_path, "w", newline="")
        self._centroid_writer = csv.writer(self._centroid_file)
        self._centroid_writer.writerow([
            "cluster_id", "map_update", "x", "y", "z", "is_active",
            "eig_0", "eig_1", "eig_2",
            "evec0_x", "evec0_y", "evec0_z",
            "evec1_x", "evec1_y", "evec1_z",
            "evec2_x", "evec2_y", "evec2_z",
        ])
        self._centroid_file.flush()

    @property
    def filepath(self) -> str:
        return self._path

    @property
    def run_dir(self) -> str:
        return self._run_dir

    def log(self, cluster_id: int, snapshot, is_stable: bool = False,
            map_update: int = 0) -> None:
        """
        Append one row for a cluster's stability snapshot.

        Args:
            cluster_id:  The tracked cluster ID.
            snapshot:    A StabilitySnapshot instance.
            is_stable:   True if cluster passed planarity filter (is_compression_candidate).
            map_update:  Current map update count — used to correlate LC events with
                         the centroid_log timeline.
        """
        # Compute cross-product magnitudes against previous snapshot
        if cluster_id in self._prev_evecs:
            cross = compute_eigenvector_stability(
                snapshot.pca_eigenvectors, self._prev_evecs[cluster_id]
            )
        else:
            cross = [0.0, 0.0, 0.0]

        self._prev_evecs[cluster_id] = snapshot.pca_eigenvectors

        evecs = snapshot.pca_eigenvectors  # (3, 3) — columns are principal axes
        row = [
            cluster_id,
            snapshot.loop_closure_count,
            map_update,
            f"{snapshot.pca_eigenvalues[0]:.6f}",
            f"{snapshot.pca_eigenvalues[1]:.6f}",
            f"{snapshot.pca_eigenvalues[2]:.6f}",
            f"{evecs[0, 0]:.6f}", f"{evecs[1, 0]:.6f}", f"{evecs[2, 0]:.6f}",  # evec0
            f"{evecs[0, 1]:.6f}", f"{evecs[1, 1]:.6f}", f"{evecs[2, 1]:.6f}",  # evec1
            f"{evecs[0, 2]:.6f}", f"{evecs[1, 2]:.6f}", f"{evecs[2, 2]:.6f}",  # evec2
            f"{cross[0]:.6f}",
            f"{cross[1]:.6f}",
            f"{cross[2]:.6f}",
            f"{snapshot.bbox_dims[0]:.4f}",
            f"{snapshot.bbox_dims[1]:.4f}",
            f"{snapshot.bbox_dims[2]:.4f}",
            snapshot.octant_occupancy,
            f"{snapshot.confidence:.4f}",
            int(is_stable),
        ]
        self._writer.writerow(row)
        self._file.flush()

    def log_centroid(self, cluster_id: int, map_update: int, centroid, is_active: bool,
                     eigenvalues=None, eigenvectors=None) -> None:
        """Append one centroid observation row.

        Args:
            eigenvalues:  (3,) array of PCA eigenvalues in descending order, or None.
            eigenvectors: (3, 3) array where columns are principal axes, or None.
        """
        row = [
            cluster_id, map_update,
            f"{centroid[0]:.4f}", f"{centroid[1]:.4f}", f"{centroid[2]:.4f}",
            int(is_active),
        ]
        if eigenvalues is not None and eigenvectors is not None:
            row += [f"{eigenvalues[i]:.6f}" for i in range(3)]
            for col in range(3):   # evec0, evec1, evec2
                row += [f"{eigenvectors[r, col]:.6f}" for r in range(3)]
        else:
            row += [""] * 12
        self._centroid_writer.writerow(row)
        self._centroid_file.flush()

    def close(self) -> None:
        """Flush and close all log files."""
        if not self._file.closed:
            self._file.flush()
            self._file.close()
        if not self._centroid_file.closed:
            self._centroid_file.flush()
            self._centroid_file.close()
