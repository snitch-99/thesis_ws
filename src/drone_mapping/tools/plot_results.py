"""
plot_results.py
---------------
Generates 4 thesis plots from one or more run directories:

  1. Centroid error over map updates (vs GT position)
  2. Number of active clusters over map updates
  3. PCA eigenvector error over loop closures (stable clusters only)
  4. Final stable cluster count per environment (with error bars across trials)

Usage:
    # Single run — plots 1, 2, 3
    python3 plot_results.py <run_dir>

    # Multiple runs of the SAME environment — adds error bars to plot 4
    python3 plot_results.py <run_dir_1> <run_dir_2> <run_dir_3>

    # All runs — aggregate plot 4 across all environments
    python3 plot_results.py --all

    # Save instead of showing
    python3 plot_results.py <run_dir> --save ./figures/
"""

import argparse
import csv
import json
import os
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np

TEST_RUNS_DIR = os.path.expanduser('~/workspaces/thesis_ws/test_runs')

plt.rcParams.update({
    'font.size': 11,
    'axes.titlesize': 13,
    'axes.labelsize': 12,
    'figure.dpi': 130,
})


# ── Data loading ──────────────────────────────────────────────────────────────

def load_run(run_dir: str) -> dict:
    meta    = json.load(open(os.path.join(run_dir, 'run_metadata.json')))
    metrics = list(csv.DictReader(open(os.path.join(run_dir, 'cluster_metrics.csv'))))
    centroids = list(csv.DictReader(open(os.path.join(run_dir, 'centroid_log.csv'))))
    return dict(meta=meta, metrics=metrics, centroids=centroids, run_dir=run_dir)


def gt_positions(run: dict) -> dict:
    """Returns {name: np.array([x, y, z])} for spawned rocks."""
    return {r['name']: np.array([r['x'], r['y'], r['z']])
            for r in run['meta']['rock_positions']}


def stable_cluster_ids(run: dict) -> set:
    seen = {}
    for row in run['metrics']:
        cid = int(row['cluster_id'])
        seen[cid] = row['is_stable']
    return {cid for cid, s in seen.items() if s == '1'}


def match_clusters_to_gt(run: dict) -> dict:
    """
    Nearest-neighbour match between final stable centroid and GT positions.
    Returns {cluster_id: gt_name}.
    """
    stable = stable_cluster_ids(run)
    gt     = gt_positions(run)
    if not stable or not gt:
        return {}

    # Final centroid per stable cluster
    final_centroids = {}
    for row in run['centroids']:
        cid = int(row['cluster_id'])
        if cid in stable:
            final_centroids[cid] = np.array([float(row['x']), float(row['y']), float(row['z'])])

    # Greedy nearest-neighbour matching
    gt_names  = list(gt.keys())
    gt_pts    = np.array([gt[n] for n in gt_names])
    matched   = {}
    used_gt   = set()
    for cid, c in final_centroids.items():
        dists = np.linalg.norm(gt_pts - c, axis=1)
        for idx in np.argsort(dists):
            if gt_names[idx] not in used_gt:
                matched[cid] = gt_names[idx]
                used_gt.add(gt_names[idx])
                break
    return matched


# ── Plot 1: Centroid error over map updates ───────────────────────────────────

def plot_centroid_error(run: dict, ax=None, save_path=None):
    gt      = gt_positions(run)
    matched = match_clusters_to_gt(run)
    stable  = stable_cluster_ids(run)

    if not matched:
        print('[plot_centroid_error] No stable clusters matched to GT.')
        return

    show = ax is None
    if ax is None:
        fig, ax = plt.subplots(figsize=(9, 4))

    # Per matched cluster: error over time
    centroid_by_cluster = defaultdict(list)
    for row in run['centroids']:
        cid = int(row['cluster_id'])
        if cid in matched:
            centroid_by_cluster[cid].append((
                int(row['map_update']),
                np.array([float(row['x']), float(row['y']), float(row['z'])])
            ))

    for cid, entries in centroid_by_cluster.items():
        gt_pt  = gt[matched[cid]]
        updates = [u for u, _ in entries]
        errors  = [np.linalg.norm(c - gt_pt) for _, c in entries]
        ax.plot(updates, errors, linewidth=1.8, label=f'Cluster {cid} → {matched[cid]}')

    ax.set_xlabel('Map Update')
    ax.set_ylabel('Centroid Error (m)')
    ax.set_title('Centroid Error vs Ground Truth over Time')
    ax.legend(fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_ylim(bottom=0)

    if show:
        plt.tight_layout()
        _show_or_save(plt.gcf(), save_path, 'centroid_error.png')


# ── Plot 2: Number of active clusters over map updates ────────────────────────

def plot_cluster_count(run: dict, ax=None, save_path=None):
    show = ax is None
    if ax is None:
        fig, ax = plt.subplots(figsize=(9, 4))

    # Count active clusters per map update
    active_count  = defaultdict(int)
    tracked_count = defaultdict(int)
    for row in run['centroids']:
        u = int(row['map_update'])
        tracked_count[u] += 1
        if row['is_active'] == '1':
            active_count[u] += 1

    updates  = sorted(tracked_count.keys())
    tracked  = [tracked_count[u] for u in updates]
    active   = [active_count[u]  for u in updates]

    ax.fill_between(updates, tracked, alpha=0.25, color='steelblue', label='Tracked (Phase 2)')
    ax.plot(updates, tracked, color='steelblue', linewidth=1.5)
    ax.fill_between(updates, active, alpha=0.35, color='seagreen', label='Active (stable centroid)')
    ax.plot(updates, active, color='seagreen', linewidth=1.5)

    # Mark Phase 3 activation (first LC)
    lc_rows = [r for r in run['metrics'] if int(r['loop_closure_count']) == 1]
    if lc_rows:
        # First update where Phase 3 was active — approximate from metrics timestamp
        ax.axvline(x=updates[len(updates)//3], color='orange', linestyle='--',
                   linewidth=1.2, label='Phase 3 activated (≈LC 1)')

    ax.set_xlabel('Map Update')
    ax.set_ylabel('Cluster Count')
    ax.set_title('Tracked vs Active Clusters over Time')
    ax.yaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    ax.legend(fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_ylim(bottom=0)

    if show:
        plt.tight_layout()
        _show_or_save(plt.gcf(), save_path, 'cluster_count.png')


# ── Plot 3: PCA eigenvector error over loop closures ─────────────────────────

def plot_pca_error(run: dict, ax=None, save_path=None):
    """Cross-product magnitude (eigenvector deviation) per LC for stable clusters."""
    show = ax is None
    if ax is None:
        fig, ax = plt.subplots(figsize=(9, 4))

    stable = stable_cluster_ids(run)
    by_cluster = defaultdict(list)
    for row in run['metrics']:
        cid = int(row['cluster_id'])
        if cid not in stable:
            continue
        lc    = int(row['loop_closure_count'])
        # Use max cross product across the 3 axes as a scalar error metric
        cross = max(float(row['cross_0']), float(row['cross_1']), float(row['cross_2']))
        by_cluster[cid].append((lc, cross))

    if not by_cluster:
        print('[plot_pca_error] No stable clusters found.')
        return

    for cid, pts in by_cluster.items():
        pts.sort()
        lcs    = [p[0] for p in pts]
        errors = [p[1] for p in pts]
        ax.plot(lcs, errors, marker='o', linewidth=1.8, markersize=5,
                label=f'Cluster {cid}')

    ax.axhline(y=0.1736, color='red', linestyle='--', linewidth=1,
               label='Threshold sin(10°)')
    ax.set_xlabel('Loop Closure #')
    ax.set_ylabel('Max Eigenvector Cross-Product')
    ax.set_title('PCA Eigenvector Stability over Loop Closures')
    ax.set_xticks(sorted({p[0] for pts in by_cluster.values() for p in pts}))
    ax.legend(fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_ylim(bottom=0)

    if show:
        plt.tight_layout()
        _show_or_save(plt.gcf(), save_path, 'pca_error.png')


# ── Plot 4: Final stable cluster count per environment (with error bars) ──────

def plot_detection_rate(all_runs: list, save_path=None):
    """
    Groups runs by num_rocks, computes mean ± std of stable output count.
    each run in all_runs is a dict from load_run().
    """
    by_env = defaultdict(list)
    for run in all_runs:
        n = run['meta']['num_rocks']
        # Count unique stable cluster IDs
        stable = len(stable_cluster_ids(run))
        by_env[n].append(stable)

    envs      = sorted(by_env.keys())
    means     = [np.mean(by_env[n]) for n in envs]
    stds      = [np.std(by_env[n])  for n in envs]
    gt_counts = envs   # ground truth = NUM_ROCKS

    fig, ax = plt.subplots(figsize=(10, 5))

    x = np.arange(len(envs))
    ax.bar(x - 0.2, gt_counts, 0.35, label='Ground Truth (rocks spawned)',
           color='steelblue', alpha=0.7, edgecolor='black')
    ax.bar(x + 0.2, means, 0.35, yerr=stds, label='Detected (mean ± std)',
           color='seagreen', alpha=0.85, edgecolor='black',
           capsize=5, error_kw=dict(elinewidth=1.5, ecolor='black'))

    ax.set_xticks(x)
    ax.set_xticklabels([f'ENV {n}\n({n} rock{"s" if n>1 else ""})' for n in envs])
    ax.set_ylabel('Cluster Count')
    ax.set_title('Detected Stable Clusters vs Ground Truth per Environment')
    ax.yaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    ax.legend(fontsize=10)
    ax.grid(axis='y', linestyle='--', alpha=0.5)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    _show_or_save(fig, save_path, 'detection_rate.png')


# ── Helpers ───────────────────────────────────────────────────────────────────

def _show_or_save(fig, save_path, filename):
    if save_path:
        os.makedirs(save_path, exist_ok=True)
        out = os.path.join(save_path, filename)
        fig.savefig(out, bbox_inches='tight')
        print(f'Saved → {out}')
        plt.close(fig)
    else:
        plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dirs', nargs='*')
    parser.add_argument('--all',  action='store_true', help='Use all runs in test_runs/')
    parser.add_argument('--save', type=str, default=None, help='Directory to save figures')
    args = parser.parse_args()

    if args.all:
        dirs = sorted([
            os.path.join(TEST_RUNS_DIR, d)
            for d in os.listdir(TEST_RUNS_DIR)
            if os.path.isdir(os.path.join(TEST_RUNS_DIR, d))
            and os.path.exists(os.path.join(TEST_RUNS_DIR, d, 'run_metadata.json'))
        ])
    else:
        dirs = args.run_dirs

    if not dirs:
        parser.print_help()
        sys.exit(1)

    all_runs = [load_run(d) for d in dirs]

    # Plots 1–3: per-run (use first run if multiple provided)
    primary = all_runs[0]
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle(
        f'Run: {os.path.basename(dirs[0])}  '
        f'(NUM_ROCKS={primary["meta"]["num_rocks"]}, Trial {primary["meta"]["trial_id"]})',
        fontsize=13, fontweight='bold'
    )
    plot_centroid_error(primary, ax=axes[0])
    plot_cluster_count(primary,  ax=axes[1])
    plot_pca_error(primary,      ax=axes[2])
    plt.tight_layout()
    _show_or_save(fig, args.save, 'run_summary.png')

    # Plot 4: detection rate across all provided runs
    plot_detection_rate(all_runs, save_path=args.save)


if __name__ == '__main__':
    main()
