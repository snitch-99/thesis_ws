"""
Phase 1 — RANSAC Ground Removal
--------------------------------
Fits a ground plane z = ax + by + c using RANSAC and removes inlier points.

Caching: if the cached plane still fits >= PLANE_REUSE_THRESHOLD of the
incoming points, skip RANSAC and reuse the cached coefficients.

Public API:
    remove_ground(points, plane_cache, params) -> (non_ground, plane_cache)
    get_ground_normal(plane_cache)             -> np.ndarray | None
"""

import numpy as np
from sklearn.linear_model import RANSACRegressor


# ── Defaults (overridden by params dict passed from cluster_tracker) ──────────
_DEFAULTS = dict(
    ransac_dist_threshold = 0.1,
    ransac_min_samples    = 3,
    ransac_max_trials     = 1000,
    plane_reuse_threshold = 0.80,
)


def _inlier_ratio(plane_coef: np.ndarray, points: np.ndarray, threshold: float) -> float:
    a, b, c   = plane_coef
    z_pred    = a * points[:, 0] + b * points[:, 1] + c
    residuals = np.abs(points[:, 2] - z_pred)
    return float((residuals < threshold).sum()) / len(points)


def remove_ground(
    points:      np.ndarray,
    plane_cache: list,
    params:      dict = None,
) -> tuple:
    """
    Remove ground plane inliers from a point cloud.

    Args:
        points:      (N, 3) XYZ array.
        plane_cache: Mutable list used as cache. Pass the same list across calls.
                     [] = no cache yet; [coef] = cached plane coefficients (3,).
        params:      Dict with Phase 1 keys (from cluster_tracker constants).
                     Falls back to _DEFAULTS if None or key missing.

    Returns:
        (non_ground, plane_cache)
        non_ground: (M, 3) array with ground points removed.
        plane_cache: updated cache (same list, mutated in place).
    """
    p = {**_DEFAULTS, **(params or {})}

    if len(points) < p['ransac_min_samples']:
        return points, plane_cache

    # ── Try to reuse cached plane ──────────────────────────────────────────────
    if len(plane_cache) > 0:
        ratio = _inlier_ratio(plane_cache[0], points, p['ransac_dist_threshold'])
        if ratio >= p['plane_reuse_threshold']:
            a, b, c       = plane_cache[0]
            z_pred        = a * points[:, 0] + b * points[:, 1] + c
            non_ground    = points[np.abs(points[:, 2] - z_pred) >= p['ransac_dist_threshold']]
            return non_ground, plane_cache

    # ── Refit RANSAC ──────────────────────────────────────────────────────────
    ransac = RANSACRegressor(
        max_trials        = p['ransac_max_trials'],
        residual_threshold= p['ransac_dist_threshold'],
        min_samples       = p['ransac_min_samples'],
        random_state      = 42,
    )
    try:
        ransac.fit(points[:, :2], points[:, 2])
    except ValueError:
        return points, plane_cache

    coef = np.array([
        ransac.estimator_.coef_[0],
        ransac.estimator_.coef_[1],
        ransac.estimator_.intercept_,
    ])

    if len(plane_cache) == 0:
        plane_cache.append(coef)
    else:
        plane_cache[0] = coef

    non_ground = points[~ransac.inlier_mask_]
    return non_ground, plane_cache


def get_ground_normal(plane_cache: list) -> np.ndarray:
    """
    Return the unit normal of the cached ground plane.

    The model fits z = ax + by + c, so the unnormalized normal is [-a, -b, 1].
    Returns None if no plane has been cached yet.
    """
    if not plane_cache:
        return None
    a, b, _ = plane_cache[0]
    n = np.array([-a, -b, 1.0])
    return n / np.linalg.norm(n)
