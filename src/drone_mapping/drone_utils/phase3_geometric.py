"""
Phase 3 — Geometric Tracking (Loop Closure Gated)
---------------------------------------------------
Runs at each loop closure. For every active cluster from Phase 2:
  1. Compute PCA, BBOX, octant occupancy.
  2. Match the cluster to its previous snapshot using eigenvector
     cross-product similarity (same approach as convergence check).
  3. After PHASE3_LC_WINDOW loop closures, apply planarity filter:
       - If the smallest eigenvector (evec2) aligns with the RANSAC
         ground normal → cluster is a ground remnant → discard.
  4. Compute a confidence score per cluster:
       confidence = lc_matched_count / PHASE3_LC_WINDOW

Public API:
    update(tracked, ground_normal, lc_count, params) -> tracked
    StabilitySnapshot  — dataclass for one loop closure observation
    is_planar(evecs, ground_normal, threshold)        -> bool
"""

import numpy as np
from dataclasses import dataclass


# ── Defaults (overridden by params dict from cluster_tracker) ─────────────────
_DEFAULTS = dict(
    phase3_lc_window    = 5,
    min_points_for_pca  = 50,
    convergence_window  = 3,
    pca_cross_thr       = 0.1736,   # sin(10°)
    bbox_dim_thr        = 0.25,
    octant_thr          = 1,
    planarity_thr       = 0.1736,   # sin(10°) — alignment with ground normal
)


# ── Data structures ───────────────────────────────────────────────────────────

@dataclass
class StabilitySnapshot:
    """One loop closure observation for a cluster."""
    loop_closure_count: int
    pca_eigenvalues:    np.ndarray   # (3,) descending
    pca_eigenvectors:   np.ndarray   # (3,3) columns = principal axes
    bbox_dims:          np.ndarray   # (3,) axis-aligned extents
    octant_occupancy:   int          # occupied octants out of 8
    confidence:         float = 0.0  # filled after PHASE3_LC_WINDOW


# ── PCA helpers ───────────────────────────────────────────────────────────────

def compute_pca(points: np.ndarray) -> tuple:
    """
    PCA on (N, 3) points.
    Returns (eigenvalues (3,) descending, eigenvectors (3,3) columns = axes).
    """
    cov = np.cov(points.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    idx          = np.argsort(eigenvalues)[::-1]
    return eigenvalues[idx], eigenvectors[:, idx]


def compute_bbox_dims(points: np.ndarray) -> np.ndarray:
    """Axis-aligned bounding box extents (3,)."""
    return points.max(axis=0) - points.min(axis=0)


def compute_octant_occupancy(points: np.ndarray, centroid: np.ndarray) -> int:
    """Count octants (of 8) around centroid that contain at least one point."""
    signs      = (points - centroid >= 0).astype(np.int8)
    octant_ids = signs @ np.array([4, 2, 1], dtype=np.int8)
    return int(np.unique(octant_ids).shape[0])


def eigenvector_cross(evecs_a: np.ndarray, evecs_b: np.ndarray) -> np.ndarray:
    """
    Per-axis cross product magnitude between two sets of eigenvectors.
    Returns (3,) — values near 0 mean axes are aligned (similar orientation).
    """
    result = np.empty(3)
    for i in range(3):
        result[i] = np.linalg.norm(np.cross(evecs_a[:, i], evecs_b[:, i]))
    return result


def is_planar(evecs: np.ndarray, eigenvalues: np.ndarray,
              ground_normal: np.ndarray,
              angle_thr: float, ratio_thr: float = 1e-3) -> bool:
    """
    True if the cluster is a ground-like flat plane.

    Requires BOTH conditions:
      1. evec2 (smallest axis) aligned with ground normal: cross < angle_thr
      2. eig2 / eig0 < ratio_thr — negligible thickness relative to spread

    A wide-but-tall rock satisfies condition 1 but fails condition 2,
    so it is correctly kept.
    """
    if ground_normal is None:
        return False
    evec2 = evecs[:, 2]
    cross = np.linalg.norm(np.cross(evec2, ground_normal))
    if cross < angle_thr:                                          # evec2 aligned with ground normal
        ratio = eigenvalues[2] / eigenvalues[0] if eigenvalues[0] > 0 else 0.0
        if ratio < ratio_thr:                                      # truly flat — no vertical extent
            return True
    return False


def check_convergence(
    history:          list,
    window:           int,
    pca_cross_thresh: float,
    bbox_thresh:      float,
    octant_thresh:    int,
) -> bool:
    """
    True if at least `window` consecutive snapshot pairs (anywhere in history)
    all pass stability criteria. One noisy LC does not disqualify the cluster.
    """
    if len(history) < 2:
        return False
    passing = 0
    for i in range(1, len(history)):
        curr, prev = history[i], history[i - 1]
        if np.any(eigenvector_cross(curr.pca_eigenvectors, prev.pca_eigenvectors) >= pca_cross_thresh):
            continue
        if np.any(np.abs(curr.bbox_dims - prev.bbox_dims) >= bbox_thresh):
            continue
        if abs(curr.octant_occupancy - prev.octant_occupancy) >= octant_thresh:
            continue
        passing += 1
    return passing >= window


# ── Public API ────────────────────────────────────────────────────────────────

def update(
    tracked:        dict,
    ground_normal:  np.ndarray,
    lc_count:       int,
    params:         dict = None,
) -> dict:
    """
    Process all active tracked clusters at a loop closure event.

    For each active cluster:
      - Compute PCA, BBOX, octant occupancy.
      - Match against previous snapshot using eigenvector cross product.
      - Increment lc_matched_count.
      - After PHASE3_LC_WINDOW closures, apply planarity filter and
        set confidence score.

    Args:
        tracked:       dict[int, TrackedCluster] from Phase 2.
        ground_normal: Unit normal of ground plane from Phase 1.
        lc_count:      Current loop closure count (1-indexed).
        params:        Dict with Phase 3 keys from cluster_tracker constants.

    Returns:
        Updated tracked dict (planar clusters marked, confidence set).
    """
    p = {**_DEFAULTS, **(params or {})}

    for tc in tracked.values():
        if not tc.is_active:
            continue
        if len(tc.points) < p['min_points_for_pca']:
            continue

        # ── Compute geometry ──────────────────────────────────────────────────
        eigenvalues, eigenvectors = compute_pca(tc.points)
        bbox_dims       = compute_bbox_dims(tc.points)
        octant_occupancy= compute_octant_occupancy(tc.points, tc.centroid)

        # ── Match against previous snapshot using eigenvector cross product ───
        matched = True
        if len(tc.stability_history) > 0:
            prev_evecs = tc.stability_history[-1].pca_eigenvectors
            cross      = eigenvector_cross(eigenvectors, prev_evecs)
            matched    = bool(np.all(cross < p['pca_cross_thr']))

        if matched:
            tc.lc_matched_count += 1

        # ── Record snapshot ───────────────────────────────────────────────────
        snapshot = StabilitySnapshot(
            loop_closure_count = lc_count,
            pca_eigenvalues    = eigenvalues,
            pca_eigenvectors   = eigenvectors,
            bbox_dims          = bbox_dims,
            octant_occupancy   = octant_occupancy,
        )
        tc.stability_history.append(snapshot)

        # ── Convergence check ─────────────────────────────────────────────────
        tc.is_compression_candidate = check_convergence(
            tc.stability_history,
            window           = p['convergence_window'],
            pca_cross_thresh = p['pca_cross_thr'],
            bbox_thresh      = p['bbox_dim_thr'],
            octant_thresh    = p['octant_thr'],
        )

        # ── Rolling confidence (updated every LC) + planarity filter ─────────
        confidence = tc.lc_matched_count / lc_count
        snapshot.confidence = confidence
        tc.stability_history[-1] = snapshot

        # Apply planarity filter once enough observations have accumulated
        if lc_count >= p['phase3_lc_window']:
            if is_planar(eigenvectors, eigenvalues, ground_normal,
                         p['planarity_thr'], p['planarity_ratio_thr']):
                tc.is_compression_candidate = False

    return tracked
