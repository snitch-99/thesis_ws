"""
Phase 2 — DBSCAN Clustering + Centroid Tracking
-------------------------------------------------
Takes the non-ground point cloud from Phase 1 and:
  1. Runs DBSCAN to find raw clusters.
  2. Drops clusters below MIN_CLUSTER_SIZE points.
  3. Matches clusters to tracked IDs using greedy nearest-centroid.
  4. Promotes clusters to 'active' once their centroid is consistent
     over CENTROID_WINDOW frames.

Public API:
    cluster_and_track(non_ground, state, params) -> state
    TrackedCluster   — dataclass for a tracked rock candidate
"""

import numpy as np
from collections import deque
from dataclasses import dataclass, field
from sklearn.cluster import DBSCAN


# ── Defaults (overridden by params dict from cluster_tracker) ─────────────────
_DEFAULTS = dict(
    dbscan_eps               = 0.1,
    dbscan_min_samples       = 30,
    min_cluster_size         = 500,
    max_match_distance       = 2.0,
    max_missing_frames       = 10,
    centroid_window          = 10,
    centroid_consistency_thr = 0.5,
)


# ── Data structures ───────────────────────────────────────────────────────────

@dataclass
class TrackedCluster:
    """A rock candidate being tracked across map updates."""
    cluster_id:     int
    centroid:       np.ndarray          # (3,) current centroid
    points:         np.ndarray          # (N, 3) latest frame points
    frames_seen:    int = 1
    frames_missing: int = 0
    is_active:      bool = False        # True once centroid is consistent
    centroid_history: object = field(default_factory=lambda: deque(maxlen=10))
    # Phase 3 fields (populated by phase3_geometric)
    stability_history:        list = field(default_factory=list)
    is_compression_candidate: bool = False
    lc_matched_count:         int  = 0   # loop closures where this cluster was matched


@dataclass
class Phase2State:
    """All mutable state for Phase 2, passed across callbacks."""
    tracked:  dict = field(default_factory=dict)   # int -> TrackedCluster
    memory:   dict = field(default_factory=dict)   # int -> centroid (long-term)
    next_id:  int  = 0
    map_update_count: int = 0
    # Funnel counters (cumulative across all frames)
    raw_dbscan_count:  int = 0   # total DBSCAN clusters found (before size filter)
    size_dropped_count: int = 0  # total dropped for being < min_cluster_size


# ── Internal helpers ──────────────────────────────────────────────────────────

def _centroid(points: np.ndarray) -> np.ndarray:
    return points.mean(axis=0)


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def _check_centroid_consistency(cluster: TrackedCluster, window: int, threshold: float) -> bool:
    """True if all centroids in the history window are within threshold of their mean."""
    if len(cluster.centroid_history) < window:
        return False
    positions = np.array(cluster.centroid_history)
    mean      = positions.mean(axis=0)
    return float(np.linalg.norm(positions - mean, axis=1).max()) < threshold


# ── Public API ────────────────────────────────────────────────────────────────

def cluster_and_track(
    non_ground:       np.ndarray,
    state:            Phase2State,
    params:           dict = None,
    lock_new_clusters: bool = False,
) -> Phase2State:
    """
    Run DBSCAN on non-ground points, filter small clusters, and update tracking.

    Args:
        non_ground:        (M, 3) points after Phase 1 ground removal.
        state:             Phase2State — mutated in place and returned.
        params:            Dict with Phase 2 keys from cluster_tracker constants.
        lock_new_clusters: If True (Phase 3 active), do not assign new IDs to
                           unmatched clusters.

    Returns:
        Updated Phase2State.
    """
    p = {**_DEFAULTS, **(params or {})}
    state.map_update_count += 1

    if len(non_ground) == 0:
        _age_unmatched(state, set(), p['max_missing_frames'])
        return state

    # ── Step 1: DBSCAN ────────────────────────────────────────────────────────
    labels = DBSCAN(
        eps        = p['dbscan_eps'],
        min_samples= p['dbscan_min_samples'],
    ).fit_predict(non_ground)

    # ── Step 2: Size filter ───────────────────────────────────────────────────
    raw_clusters = []
    for label in set(labels):
        if label == -1:
            continue
        state.raw_dbscan_count += 1
        pts = non_ground[labels == label]
        if len(pts) >= p['min_cluster_size']:
            raw_clusters.append(pts)
        else:
            state.size_dropped_count += 1

    # ── Step 3: Greedy centroid matching ──────────────────────────────────────
    new_centroids       = [_centroid(c) for c in raw_clusters]
    matched_tracked_ids = set()
    matched_memory_ids  = set()
    updated             = {}

    for pts, new_c in zip(raw_clusters, new_centroids):
        best_id, best_dist = _find_best_match(
            new_c, state.tracked, matched_tracked_ids, p['max_match_distance']
        )

        if best_id is not None:
            # Matched active tracked cluster
            tc = state.tracked[best_id]
            tc.centroid = new_c
            tc.centroid_history.append(new_c.copy())
            tc.points         = pts
            tc.frames_seen   += 1
            tc.frames_missing = 0
            updated[best_id]  = tc
            matched_tracked_ids.add(best_id)
            state.memory[best_id] = new_c
            continue

        # Try long-term memory
        best_id, best_dist = _find_best_match(
            new_c, state.memory, matched_tracked_ids | matched_memory_ids,
            p['max_match_distance'], keys_only=True
        )

        if best_id is not None and best_id not in updated:
            matched_memory_ids.add(best_id)
            tc = TrackedCluster(
                cluster_id=best_id,
                centroid=new_c,
                points=pts,
            )
            tc.centroid_history.append(new_c.copy())
            updated[best_id]      = tc
            state.memory[best_id] = new_c
            continue

        # New cluster — skip if Phase 3 is active
        if lock_new_clusters:
            continue
        tc = TrackedCluster(
            cluster_id=state.next_id,
            centroid=new_c,
            points=pts,
        )
        tc.centroid_history.append(new_c.copy())
        updated[state.next_id]      = tc
        state.memory[state.next_id] = new_c
        state.next_id += 1

    # ── Step 4: Age unmatched clusters ────────────────────────────────────────
    _age_unmatched(state, matched_tracked_ids, p['max_missing_frames'], updated)

    # ── Step 5: Centroid consistency check ────────────────────────────────────
    for tc in updated.values():
        tc.is_active = _check_centroid_consistency(
            tc, p['centroid_window'], p['centroid_consistency_thr']
        )

    state.tracked = updated
    return state


# ── Internal helpers ──────────────────────────────────────────────────────────

def _find_best_match(
    new_centroid: np.ndarray,
    candidates:   dict,
    excluded_ids: set,
    max_dist:     float,
    keys_only:    bool = False,
) -> tuple:
    """Return (best_id, best_dist) from candidates, ignoring excluded_ids."""
    best_id, best_dist = None, float('inf')
    for cid, val in candidates.items():
        if cid in excluded_ids:
            continue
        c = val if keys_only else val.centroid
        d = _dist(new_centroid, c)
        if d < best_dist:
            best_dist = d
            best_id   = cid
    if best_id is not None and best_dist < max_dist:
        return best_id, best_dist
    return None, float('inf')


def _age_unmatched(
    state:              Phase2State,
    matched_ids:        set,
    max_missing_frames: int,
    updated:            dict = None,
):
    """Increment frames_missing for unmatched clusters; drop if over limit."""
    if updated is None:
        updated = {}
    for tid, tc in state.tracked.items():
        if tid not in matched_ids and tid not in updated:
            tc.frames_missing += 1
            if tc.frames_missing <= max_missing_frames:
                updated[tid] = tc
            # Centroid stays in memory forever — enables resurrection
