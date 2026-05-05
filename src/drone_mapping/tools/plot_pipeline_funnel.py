"""
plot_pipeline_funnel.py
-----------------------
Reads phase_stats.json from one or more run directories and produces a
pipeline funnel bar chart showing how clusters are filtered at each stage.

Usage:
    python3 plot_pipeline_funnel.py <run_dir>
    python3 plot_pipeline_funnel.py <run_dir_1> <run_dir_2> ...   # overlay / average
    python3 plot_pipeline_funnel.py --all                         # all runs in test_runs/
"""

import argparse
import json
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

TEST_RUNS_DIR = os.path.expanduser('~/workspaces/thesis_ws/test_runs')

# ── Pipeline stage labels and colors ─────────────────────────────────────────
STAGES = [
    ('Raw DBSCAN\nDetections',    '#aec6cf'),   # Phase 2 input (cumulative)
    ('After Size\nFilter',        '#fdfd96'),   # Phase 2 — dropped < 500 pts
    ('Unique IDs\nTracked',       '#ffb347'),   # Phase 2 — unique IDs assigned
    ('Active\nClusters',          '#77dd77'),   # Phase 2 — passed centroid consistency
    ('Reached\nPhase 3',          '#87ceeb'),   # Phase 3 input
    ('Stable\nOutput',            '#4caf50'),   # Phase 3 — passed planarity
]

DROPPED_COLOR = '#ff6961'   # red — for removed bars


def load_stats(run_dir: str) -> dict:
    path = os.path.join(run_dir, 'phase_stats.json')
    if not os.path.exists(path):
        raise FileNotFoundError(f'No phase_stats.json in {run_dir}')
    with open(path) as f:
        return json.load(f)


def stats_to_funnel(s: dict) -> list:
    """Extract ordered funnel values from a stats dict."""
    after_size = s['phase2_raw_dbscan_detections'] - s['phase2_size_dropped']
    return [
        s['phase2_raw_dbscan_detections'],
        after_size,
        s['phase2_unique_ids_tracked'],
        s['phase2_active_clusters'],
        s['phase3_reached'],
        s['phase3_stable_output'],
    ]


def plot_single(stats: dict, title: str = 'Pipeline Funnel'):
    funnel = stats_to_funnel(stats)
    labels = [s[0] for s in STAGES]
    colors = [s[1] for s in STAGES]

    fig, ax = plt.subplots(figsize=(12, 6))

    bars = ax.bar(labels, funnel, color=colors, edgecolor='black', linewidth=0.8, zorder=3)

    # Annotate drop counts between stages
    for i in range(1, len(funnel)):
        dropped = funnel[i - 1] - funnel[i]
        if dropped > 0:
            mid_x = i - 0.5
            mid_y = max(funnel[i - 1], funnel[i]) * 1.02
            ax.annotate(
                f'−{dropped}',
                xy=(mid_x, mid_y),
                fontsize=9, color=DROPPED_COLOR, ha='center', fontweight='bold'
            )

    # Value labels on bars
    for bar, val in zip(bars, funnel):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(funnel) * 0.01,
            str(val), ha='center', va='bottom', fontsize=10, fontweight='bold'
        )

    ax.set_ylabel('Cluster Count', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(axis='y', linestyle='--', alpha=0.5, zorder=0)
    ax.set_ylim(0, max(funnel) * 1.18)

    # Phase labels as background bands
    phase2_patch = mpatches.Patch(color='#fffde7', label='Phase 2 — DBSCAN + Tracking')
    phase3_patch = mpatches.Patch(color='#e8f5e9', label='Phase 3 — Geometric Filtering')
    ax.axvspan(-0.5, 3.5, alpha=0.15, color='#fff176', zorder=0)
    ax.axvspan(3.5, 5.5, alpha=0.15, color='#a5d6a7', zorder=0)
    ax.legend(handles=[phase2_patch, phase3_patch], loc='upper right', fontsize=10)

    plt.tight_layout()
    return fig


def plot_multi(all_stats: list, labels: list, title: str = 'Pipeline Funnel — All Runs'):
    """Grouped bar chart comparing multiple runs."""
    n_runs   = len(all_stats)
    n_stages = len(STAGES)
    x        = np.arange(n_stages)
    width    = 0.8 / n_runs

    fig, ax = plt.subplots(figsize=(14, 6))

    for i, (stats, run_label) in enumerate(zip(all_stats, labels)):
        funnel = stats_to_funnel(stats)
        offset = (i - n_runs / 2 + 0.5) * width
        ax.bar(x + offset, funnel, width, label=run_label, alpha=0.85, edgecolor='black', linewidth=0.6)

    ax.set_xticks(x)
    ax.set_xticklabels([s[0] for s in STAGES], fontsize=10)
    ax.set_ylabel('Cluster Count', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(fontsize=8, ncol=min(n_runs, 5))
    ax.grid(axis='y', linestyle='--', alpha=0.5)

    plt.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dirs', nargs='*', help='Run directories to plot')
    parser.add_argument('--all', action='store_true', help='Plot all runs in test_runs/')
    parser.add_argument('--save', type=str, default=None, help='Save to file instead of showing')
    args = parser.parse_args()

    if args.all:
        run_dirs = sorted([
            os.path.join(TEST_RUNS_DIR, d)
            for d in os.listdir(TEST_RUNS_DIR)
            if os.path.isdir(os.path.join(TEST_RUNS_DIR, d))
               and os.path.exists(os.path.join(TEST_RUNS_DIR, d, 'phase_stats.json'))
        ])
        if not run_dirs:
            print('No runs with phase_stats.json found.')
            sys.exit(1)
    else:
        run_dirs = args.run_dirs
        if not run_dirs:
            parser.print_help()
            sys.exit(1)

    all_stats = [load_stats(d) for d in run_dirs]
    run_labels = [os.path.basename(d) for d in run_dirs]

    if len(all_stats) == 1:
        # Load num_rocks from metadata if available
        meta_path = os.path.join(run_dirs[0], 'run_metadata.json')
        title = f'Pipeline Funnel — {run_labels[0]}'
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                meta = json.load(f)
            title += f'  (NUM_ROCKS={meta["num_rocks"]}, Trial {meta["trial_id"]})'
        fig = plot_single(all_stats[0], title=title)
    else:
        fig = plot_multi(all_stats, run_labels)

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches='tight')
        print(f'Saved to {args.save}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
