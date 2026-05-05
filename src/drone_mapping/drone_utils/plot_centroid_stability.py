"""
plot_centroid_stability.py
--------------------------
Plot centroid X, Y, Z over map update events for each stable cluster.

Usage:
    python3 plot_centroid_stability.py <path/to/run_dir>
    python3 plot_centroid_stability.py          # uses latest run in ~/workspaces/thesis_ws/test_runs/
"""

import sys
import os
import glob

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Constants ──
BASE_OUTPUT_DIR = os.path.expanduser("~/workspaces/thesis_ws/test_runs")
AXES_LABELS     = ["X (m)", "Y (m)", "Z (m)"]
AXES_KEYS       = ["x", "y", "z"]
FIGURE_DPI      = 150
ACTIVE_ALPHA    = 1.0
INACTIVE_ALPHA  = 0.35

COLORS = [
    "#e6194b",  # red
    "#3cb44b",  # green
    "#4363d8",  # blue
    "#00ced1",  # cyan
    "#f58231",  # orange
    "#911eb4",  # purple
    "#f032e6",  # magenta
    "#a9a9a9",  # grey
]


def _latest_run_dir(base: str) -> str:
    runs = sorted(glob.glob(os.path.join(base, "run_*")))
    if not runs:
        raise FileNotFoundError(f"No run directories found in {base}")
    return runs[-1]


def load_centroid_log(run_dir: str) -> pd.DataFrame:
    path = os.path.join(run_dir, "centroid_log.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"centroid_log.csv not found in {run_dir}.\n"
            "Run the simulation with the updated cluster_tracker first."
        )
    return pd.read_csv(path)


def plot_centroid_stability(run_dir: str):
    df = load_centroid_log(run_dir)

    cluster_ids = sorted(df["cluster_id"].unique())
    n_clusters  = len(cluster_ids)
    color_map   = {cid: COLORS[i % len(COLORS)] for i, cid in enumerate(cluster_ids)}

    # Only plot clusters that were eventually promoted to active
    stable_ids = df[df["is_active"] == 1]["cluster_id"].unique()
    if len(stable_ids) == 0:
        print("No stable clusters found in the log. Run the simulation longer.")
        return

    # ── Per-axis subplots, one line per stable cluster ──
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle("Centroid Stability Over Map Updates", fontsize=14, fontweight="bold")

    for ax, axis_key, axis_label in zip(axes, AXES_KEYS, AXES_LABELS):
        for cid in stable_ids:
            cdf   = df[df["cluster_id"] == cid].sort_values("map_update")
            color = color_map[cid]

            # Split into pre-active (unstable) and active (stable) segments
            pre    = cdf[cdf["is_active"] == 0]
            active = cdf[cdf["is_active"] == 1]

            if len(pre):
                ax.plot(pre["map_update"], pre[axis_key],
                        color=color, alpha=INACTIVE_ALPHA, linewidth=1.2,
                        linestyle="--")
            if len(active):
                ax.plot(active["map_update"], active[axis_key],
                        color=color, alpha=ACTIVE_ALPHA, linewidth=1.8,
                        linestyle="-", label=f"Cluster {cid}")

            # Mark the transition point (first map update where is_active=1)
            if len(active):
                t0 = active["map_update"].iloc[0]
                v0 = active[axis_key].iloc[0]
                ax.axvline(t0, color=color, alpha=0.25, linewidth=0.8, linestyle=":")
                ax.scatter([t0], [v0], color=color, s=40, zorder=5)

        ax.set_ylabel(axis_label, fontsize=11)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Map Update Event", fontsize=11)

    # Legend
    handles = [
        mpatches.Patch(color=color_map[cid], label=f"Cluster {cid}")
        for cid in stable_ids
    ]
    handles += [
        plt.Line2D([0], [0], color="k", linestyle="--", alpha=0.5, label="Pre-stabilisation"),
        plt.Line2D([0], [0], color="k", linestyle="-",  label="Stable (active)"),
    ]
    axes[0].legend(handles=handles, loc="upper right", fontsize=9, ncol=2)

    plt.tight_layout()
    out_path = os.path.join(run_dir, "centroid_stability.png")
    plt.savefig(out_path, dpi=FIGURE_DPI, bbox_inches="tight")
    plt.close()
    print(f"Saved: {out_path}")

    # ── Per-cluster 3-axis panel ──
    for cid in stable_ids:
        cdf   = df[df["cluster_id"] == cid].sort_values("map_update")
        color = color_map[cid]

        fig2, axes2 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        fig2.suptitle(f"Cluster {cid} — Centroid Convergence", fontsize=13, fontweight="bold")

        for ax, axis_key, axis_label in zip(axes2, AXES_KEYS, AXES_LABELS):
            pre    = cdf[cdf["is_active"] == 0]
            active = cdf[cdf["is_active"] == 1]

            if len(pre):
                ax.plot(pre["map_update"], pre[axis_key],
                        color=color, alpha=INACTIVE_ALPHA, linewidth=1.4,
                        linestyle="--", label="Drifting")
            if len(active):
                ax.plot(active["map_update"], active[axis_key],
                        color=color, alpha=ACTIVE_ALPHA, linewidth=2.0,
                        linestyle="-", label="Stable")

            if len(active):
                t0 = active["map_update"].iloc[0]
                ax.axvline(t0, color="black", alpha=0.4, linewidth=1.0,
                           linestyle=":", label=f"Activated @ update {t0}")

            ax.set_ylabel(axis_label, fontsize=11)
            ax.legend(fontsize=8, loc="upper right")
            ax.grid(True, alpha=0.3)

        axes2[-1].set_xlabel("Map Update Event", fontsize=11)
        plt.tight_layout()

        out2 = os.path.join(run_dir, f"centroid_cluster_{cid}.png")
        plt.savefig(out2, dpi=FIGURE_DPI, bbox_inches="tight")
        plt.close()
        print(f"Saved: {out2}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        run_dir = sys.argv[1]
    else:
        run_dir = _latest_run_dir(BASE_OUTPUT_DIR)

    print(f"Using run: {run_dir}")
    plot_centroid_stability(run_dir)
