"""
Generate all Section 6.1 thesis figures.
Run with: python3 generate_figures.py
"""

import csv
import glob
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import open3d as o3d

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
TEST_RUNS_DIR = "/home/kanav/workspaces/thesis_ws/test_runs"
OUTPUT_DIR = (
    "/home/kanav/workspaces/Thesis_documents/"
    "Autonomous_Geometric_Compression_and_Environmental_Parsing_for_Extraterrestrial_Exploration/"
    "Images"
)
REP_RUN     = "/home/kanav/workspaces/thesis_ws/test_runs/run_20260408_134421"
WAYPOINTS   = "/home/kanav/workspaces/thesis_ws/src/drone_mapping/drone_utils/waypoints.csv"

DETECTION_DATA = {
    1:  [1, 1, 1],
    2:  [2, 2, 2],
    3:  [3, 3, 3],
    4:  [4, 4, 4],
    5:  [5, 4, 5],
    6:  [5, 5, 4],
    7:  [6, 6, 7],
    8:  [7, 8, 8],
    9:  [8, 8, 9],
    10: [7, 10, 10],
}

PHASE_COLORS = {"Phase 1": "#F48FB1", "Phase 2": "#90CAF9", "Phase 3": "#A5D6A7"}

STYLE = {
    "font.family":    "serif",
    "axes.titlesize": 13,
    "axes.labelsize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 10,
    "figure.dpi":     150,
}
plt.rcParams.update(STYLE)

os.makedirs(OUTPUT_DIR, exist_ok=True)


def savefig(fig, name):
    path = os.path.join(OUTPUT_DIR, f"chap_6_{name}.png")
    fig.savefig(path, dpi=300, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  Saved: chap_6_{name}.png")


# ─────────────────────────────────────────────────────────────────────────────
# FIG 1 — Detection line chart
# ─────────────────────────────────────────────────────────────────────────────
def fig_detection_chart():
    fig, ax = plt.subplots(figsize=(8, 5))

    num_rocks = sorted(DETECTION_DATA.keys())
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    markers = ["o", "s", "^"]

    for i, (color, marker) in enumerate(zip(colors, markers)):
        detected = [DETECTION_DATA[n][i] for n in num_rocks]
        ax.plot(num_rocks, detected, marker=marker, color=color,
                linewidth=2, markersize=7, label=f"Trial {i+1}")

    ax.plot(num_rocks, num_rocks, "k--", linewidth=1.5, label="Perfect detection (y = x)")

    ax.set_xlabel("Number of Rocks in Scene")
    ax.set_ylabel("Number of Rocks Detected")
    ax.set_title("Rock Detection Performance Across 30 Trials")
    ax.set_xticks(num_rocks)
    ax.set_yticks(num_rocks)
    ax.set_xlim(0.5, 10.5)
    ax.set_ylim(0, 11)
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "detection_line_chart")


# ─────────────────────────────────────────────────────────────────────────────
# FIG 2 — Cluster count evolution vs map update (representative 10-rock run)
# ─────────────────────────────────────────────────────────────────────────────
def fig_pipeline_funnel():
    # Count unique tracked clusters per map update from centroid_log
    mu_counts = {}
    with open(f"{REP_RUN}/centroid_log.csv") as f:
        for row in csv.DictReader(f):
            mu = int(row["map_update"])
            mu_counts[mu] = mu_counts.get(mu, 0) + 1

    mus    = sorted(mu_counts.keys())
    counts = [mu_counts[mu] for mu in mus]

    # First loop closure map update
    first_lc_mu = None
    with open(f"{REP_RUN}/cluster_metrics.csv") as f:
        for row in csv.DictReader(f):
            if int(row["loop_closure_count"]) == 1:
                first_lc_mu = int(row["map_update"])
                break

    num_rocks = 10  # ground truth for representative run

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(mus, counts, color="#1f77b4", linewidth=2.2, marker="o",
            markersize=4, label="Tracked cluster count")
    ax.axhline(y=num_rocks, color="black", linestyle=":", linewidth=1.5,
               label=f"Ground truth ({num_rocks} rocks)")
    if first_lc_mu is not None:
        ax.axvline(x=first_lc_mu, color="red", linestyle="--", linewidth=2,
                   label=f"First loop closure (MU = {first_lc_mu})")

    ax.set_xlabel("Map Update Event")
    ax.set_ylabel("Number of Tracked Clusters")
    ax.set_title("Cluster Count Evolution During Survey\n(10-Rock Representative Trial)")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "pipeline_funnel")


# ─────────────────────────────────────────────────────────────────────────────
# FIG 3 — Centroid error vs map update (all detected clusters)
# ─────────────────────────────────────────────────────────────────────────────
def fig_centroid_error():
    # PLY centroids as ground truth
    ply_centroids = {}
    for fpath in sorted(glob.glob(f"{REP_RUN}/clusters/*.ply")):
        cid = int(os.path.basename(fpath).replace("cluster_", "").replace(".ply", ""))
        pcd = o3d.io.read_point_cloud(fpath)
        pts = np.asarray(pcd.points)
        ply_centroids[cid] = pts.mean(axis=0)

    # First LC map_update from cluster_metrics
    first_lc_mu = None
    with open(f"{REP_RUN}/cluster_metrics.csv") as f:
        for row in csv.DictReader(f):
            if int(row["loop_closure_count"]) == 1:
                first_lc_mu = int(row["map_update"])
                break

    # Load centroid log
    cluster_data = {}
    with open(f"{REP_RUN}/centroid_log.csv") as f:
        for row in csv.DictReader(f):
            if not row["eig_0"]:
                continue
            cid = int(row["cluster_id"])
            if cid not in cluster_data:
                cluster_data[cid] = {"mus": [], "centroids": []}
            cluster_data[cid]["mus"].append(int(row["map_update"]))
            cluster_data[cid]["centroids"].append(
                np.array([float(row["x"]), float(row["y"]), float(row["z"])]))

    detected_cids = sorted(ply_centroids.keys())
    cmap = plt.get_cmap("tab10")

    fig, ax = plt.subplots(figsize=(10, 5))

    for i, cid in enumerate(detected_cids):
        if cid not in cluster_data:
            continue
        d   = cluster_data[cid]
        gt  = ply_centroids[cid]
        mus    = d["mus"]
        errors = [np.linalg.norm(c - gt) for c in d["centroids"]]
        ax.plot(mus, errors, marker="o", linewidth=1.8, markersize=4,
                color=cmap(i % 10), label=f"Cluster {cid}")

    ax.axvline(x=first_lc_mu, color="red", linestyle="--", linewidth=2,
               label=f"First Loop Closure (MU = {first_lc_mu})")

    ax.set_xlabel("Map Update Event")
    ax.set_ylabel("Centroid Error (m)")
    ax.set_title("Centroid Convergence to Final Position\n(10-Rock Representative Trial)")
    ax.legend(loc="upper right", ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "centroid_error")


# ─────────────────────────────────────────────────────────────────────────────
# FIG 4 — Eigenvector stability vs map update (all detected clusters)
# ─────────────────────────────────────────────────────────────────────────────
def fig_eigenvector_stability():
    # PLY cluster IDs
    detected_cids = sorted([
        int(os.path.basename(f).replace("cluster_", "").replace(".ply", ""))
        for f in glob.glob(f"{REP_RUN}/clusters/*.ply")
    ])

    # First LC map_update
    first_lc_mu = None
    with open(f"{REP_RUN}/cluster_metrics.csv") as f:
        for row in csv.DictReader(f):
            if int(row["loop_closure_count"]) == 1:
                first_lc_mu = int(row["map_update"])
                break

    # Load eigenvectors from centroid_log
    cluster_data = {}
    with open(f"{REP_RUN}/centroid_log.csv") as f:
        for row in csv.DictReader(f):
            if not row["eig_0"]:
                continue
            cid = int(row["cluster_id"])
            if cid not in detected_cids:
                continue
            if cid not in cluster_data:
                cluster_data[cid] = {"mus": [], "evec0s": []}
            cluster_data[cid]["mus"].append(int(row["map_update"]))
            cluster_data[cid]["evec0s"].append(np.array([
                float(row["evec0_x"]), float(row["evec0_y"]), float(row["evec0_z"])]))

    cmap = plt.get_cmap("tab10")
    threshold = np.sin(np.radians(10))

    fig, ax = plt.subplots(figsize=(10, 5))

    for i, cid in enumerate(detected_cids):
        if cid not in cluster_data:
            continue
        d = cluster_data[cid]
        mus    = d["mus"]
        evec0s = d["evec0s"]
        cross  = [0.0] + [
            np.linalg.norm(np.cross(evec0s[j], evec0s[j-1]))
            for j in range(1, len(evec0s))
        ]
        ax.plot(mus, cross, marker="o", linewidth=1.8, markersize=4,
                color=cmap(i % 10), label=f"Cluster {cid}")

    ax.axvline(x=first_lc_mu, color="red", linestyle="--", linewidth=2,
               label=f"First Loop Closure (MU = {first_lc_mu})")
    ax.axhline(y=threshold, color="black", linestyle=":", linewidth=1.5,
               label=f"Stability threshold (sin 10° = {threshold:.3f})")

    ax.set_xlabel("Map Update Event")
    ax.set_ylabel("|evec$_0$(t) × evec$_0$(t−1)|")
    ax.set_title("Eigenvector Stability Across Map Updates\n(10-Rock Representative Trial)")
    ax.legend(loc="upper right", ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "eigenvector_stability")


# ─────────────────────────────────────────────────────────────────────────────
# FIG 5 — 2D aerial view (success case, 10/10)
# ─────────────────────────────────────────────────────────────────────────────
def fig_aerial_view():
    wps = np.loadtxt(WAYPOINTS, delimiter=",", skiprows=1)

    with open(f"{REP_RUN}/run_metadata.json") as f:
        meta = json.load(f)
    gt = np.array([[r["x"], r["y"]] for r in meta["rock_positions"]])

    detected = {}
    for fpath in sorted(glob.glob(f"{REP_RUN}/clusters/*.ply")):
        cid = int(os.path.basename(fpath).replace("cluster_", "").replace(".ply", ""))
        pcd = o3d.io.read_point_cloud(fpath)
        pts = np.asarray(pcd.points)
        detected[cid] = pts.mean(axis=0)[:2]

    fig, ax = plt.subplots(figsize=(7, 7))

    ax.plot(wps[:, 0], wps[:, 1], "-", color="#AAAAAA", linewidth=1.5,
            alpha=0.8, label="Drone trajectory", zorder=1)
    ax.plot(wps[0, 0],  wps[0, 1],  "g^", markersize=11, zorder=3, label="Start")
    ax.plot(wps[-1, 0], wps[-1, 1], "rs", markersize=11, zorder=3, label="End")

    ax.scatter(gt[:, 0], gt[:, 1], s=220, marker="*", color="#FFD700",
               edgecolors="black", linewidths=0.8, zorder=4, label="Ground truth rock")

    det_xy = np.array(list(detected.values()))
    ax.scatter(det_xy[:, 0], det_xy[:, 1], s=110, marker="o", color="#1f77b4",
               edgecolors="black", linewidths=0.8, zorder=5, label="Detected centroid")

    used_gt = set()
    for cxy in det_xy:
        dists = np.linalg.norm(gt - cxy, axis=1)
        idx   = int(np.argmin(dists))
        if dists[idx] < 2.0 and idx not in used_gt:
            used_gt.add(idx)
            ax.plot([gt[idx, 0], cxy[0]], [gt[idx, 1], cxy[1]],
                    "k--", linewidth=0.8, alpha=0.4)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Aerial View — Drone Trajectory and Rock Detections\n(10/10 Detected)")
    ax.legend(loc="upper left")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "aerial_view")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Generating Section 6.1 thesis figures...")
    fig_detection_chart()
    fig_pipeline_funnel()
    fig_centroid_error()
    fig_eigenvector_stability()
    fig_aerial_view()
    print("\nDone. All figures saved to:")
    print(f"  {OUTPUT_DIR}")
