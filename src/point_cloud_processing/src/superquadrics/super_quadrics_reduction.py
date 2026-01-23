import numpy as np
import open3d as o3d
import scipy.optimize
import scipy.signal
import scipy.ndimage
import cv2
import os
import copy
from tqdm import tqdm
import sys
import os
from ems_core import Superquadric, EMSFitter

import utils
import multiprocessing
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

# SYS PATH HACKS for re-structuring
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, 'utils'))
sys.path.append(os.path.join(parent_dir, 'deviation'))

# --- CONFIGURATION ---
# FILENAME = "base_cloud.ply" 
# FILENAME = "Changed.ply"
#FILENAME = "scene_delta.ply"
FILENAME = "rock1.ply"

def create_grid(size=10, step=1.0):
    """Creates a line set grid on the XZ plane (y=0) or XY plane."""
    lines = []
    points = []
    color = [0.8, 0.8, 0.8] # Light Grey
    colors = []
    
    # X lines
    for x in np.arange(-size, size + step, step):
        points.append([x, -size, 0])
        points.append([x, size, 0])
        lines.append([len(points)-2, len(points)-1])
        colors.append(color)
        
    # Y lines
    for y in np.arange(-size, size + step, step):
        points.append([-size, y, 0])
        points.append([size, y, 0])
        lines.append([len(points)-2, len(points)-1])
        colors.append(color)
        
    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.colors = o3d.utility.Vector3dVector(colors)
    return grid

def show_window_proc(window_name, pcd_points, mesh_data=None, point_colors=None):
    """
    Worker process to show a window with GUI controls.
    mesh_data: (vertices, triangles, color)
    """
    gui.Application.instance.initialize()
    w = gui.Application.instance.create_window(window_name, 1024, 768)
    
    # Materials
    mat = rendering.MaterialRecord()
    mat.shader = "defaultLit"
    
    mat_pt = rendering.MaterialRecord()
    mat_pt.shader = "defaultUnlit"
    mat_pt.point_size = 4.0

    mat_line = rendering.MaterialRecord()
    mat_line.shader = "unlitLine"
    mat_line.line_width = 2.0

    # Scene Widget
    scene_widget = gui.SceneWidget()
    scene_widget.scene = rendering.Open3DScene(w.renderer)
    
    # 1. Add Grid & Origin
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    scene_widget.scene.add_geometry("origin", origin, mat)
    
    grid = create_grid(size=5.0, step=0.5)
    scene_widget.scene.add_geometry("grid", grid, mat_line)

    # State variables for color switching
    state = {
        "geometry_name": None,
        "target_geometry": None,
        "original_colors_vector": None, # If per-point colors exist
        "original_uniform_color": None, # If uniform color exists
        "is_point_cloud": False
    }
    
    # Track Bounds for Camera
    bbox = o3d.geometry.AxisAlignedBoundingBox()

    # 2. Add Point Cloud
    if pcd_points is not None:
        state["geometry_name"] = "point_cloud"
        state["is_point_cloud"] = True
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_points)
        
        if point_colors is not None:
             pcd.colors = o3d.utility.Vector3dVector(point_colors)
             state["original_colors_vector"] = pcd.colors
        else:
             default_c = [0.6, 0.6, 0.6]
             pcd.paint_uniform_color(default_c) 
             state["original_uniform_color"] = default_c
             
        scene_widget.scene.add_geometry(state["geometry_name"], pcd, mat_pt)
        state["target_geometry"] = pcd
        bbox += pcd.get_axis_aligned_bounding_box()

    # 3. Add Mesh (Mutually exclusive with Cloud in this app usually, but handled if exists)
    if mesh_data is not None:
        verts, faces, color = mesh_data
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(verts)
        mesh.triangles = o3d.utility.Vector3iVector(faces)
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color(color)
        
        state["geometry_name"] = "mesh"
        state["is_point_cloud"] = False
        state["original_uniform_color"] = color
        
        scene_widget.scene.add_geometry(state["geometry_name"], mesh, mat)
        state["target_geometry"] = mesh
        bbox += mesh.get_axis_aligned_bounding_box()

    # Setup Camera
    if bbox.is_empty():
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-2,-2,-2), max_bound=(2,2,2))
    scene_widget.setup_camera(60, bbox, bbox.get_center())
    
    # --- GUI Layout ---
    em = w.theme.font_size
    layout = gui.Vert(0, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))
    
    # Label
    label = gui.Label("Color Settings")
    # Font Fix
    # font_label = gui.FontDescription(gui.FontDescription.SANS_SERIF, gui.FontStyle.BOLD, 14)
    # label.font_id = gui.Application.instance.add_font(font_label)
    label.text_color = gui.Color(1.0, 1.0, 1.0)
    
    # Combobox
    combo = gui.Combobox()
    combo.add_item("Original/Default")
    combo.add_item("Red")
    combo.add_item("Green")
    combo.add_item("Blue")
    combo.add_item("Cyan")
    combo.add_item("Magenta")
    combo.add_item("Yellow")
    combo.add_item("Grey")
    
    def on_color_change(new_val, new_idx):
        if state["target_geometry"] is None: return
        
        geo_name = state["geometry_name"]
        geom = state["target_geometry"]
        
        c_map = {
            "Red": [1, 0, 0],
            "Green": [0, 1, 0],
            "Blue": [0, 0, 1],
            "Cyan": [0, 1, 1],
            "Magenta": [1, 0, 1],
            "Yellow": [1, 1, 0],
            "Grey": [0.5, 0.5, 0.5]
        }
        
        if new_val == "Original/Default":
            if state["is_point_cloud"] and state["original_colors_vector"] is not None:
                geom.colors = state["original_colors_vector"]
            elif state["original_uniform_color"] is not None:
                geom.paint_uniform_color(state["original_uniform_color"])
        else:
            color = c_map.get(new_val, [0.5, 0.5, 0.5])
            geom.paint_uniform_color(color)
            
        # Update Geometry
        scene_widget.scene.remove_geometry(geo_name)
        m = mat_pt if state["is_point_cloud"] else mat
        scene_widget.scene.add_geometry(geo_name, geom, m)
        
    combo.set_on_selection_changed(on_color_change)
    
    layout.add_child(label)
    layout.add_child(combo)
    
    w.add_child(scene_widget)
    w.add_child(layout)
    
    # Layout Callbacks
    def on_layout(layout_context):
        r = w.content_rect
        width = 17 * layout_context.theme.font_size
        layout.frame = gui.Rect(r.x, r.y, width, r.height)
        scene_widget.frame = gui.Rect(r.x + width, r.y, r.width - width, r.height)
        
    w.set_on_layout(on_layout)
    
    gui.Application.instance.run()

def main():
    pcd_dir = os.path.join(os.path.dirname(__file__), '../..', 'point_clouds')
    file_path = os.path.join(pcd_dir, FILENAME)
    
    print(f"Loading {file_path}...")
    try:
        pcd = o3d.io.read_point_cloud(file_path)
    except:
        print("Could not load file. Creating dummy cube for testing.")
        mesh = o3d.geometry.TriangleMesh.create_box()
        pcd = mesh.sample_points_poisson_disk(1000)
    
    if pcd.is_empty():
        print("Point cloud empty.")
        return

    # Initialization Comparison: BBOX only (Cleanup)
    init_strategies = ['BBOX']
    results_table = [] 
    
    viz_models = [] # To store (name, model, fitter)
    
    print("\n=== Starting Initialization Comparison (Threshold = +/- 10cm) ===")
    
    # Create valid progress bar
    pbar = tqdm(total=None, bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}] {postfix}')
    
    for strategy in init_strategies:
        print(f"\n--- Strategy: {strategy} ---")
        fitter = EMSFitter(pcd, init_type=strategy)
        # Use default w0=0.5 or maybe slightly higher? 0.5 is fair.
        fitter.w_o_prior = 0.5 
        
        # Reset pbar for this run? Or pass new one?
        # Let's make a new inner pbar for each run to be clean
        inner_pbar = tqdm(total=20, desc=f"Fitting {strategy}")
        
        # Fit
        sq_model = fitter.fit(max_iters=100, external_pbar=inner_pbar)
        inner_pbar.close()
        
        # Analyze
        num_inliers, num_outliers, _ = utils.analyze_inliers(sq_model, fitter.points, distance_threshold=0.1)
        
        # Store results
        p_str = f"[{sq_model.ax:.2f}, {sq_model.ay:.2f}, {sq_model.az:.2f}, {sq_model.e1:.2f}, {sq_model.e2:.2f}]"
        results_table.append((strategy, num_inliers, num_outliers, p_str))
        
        viz_models.append((strategy, sq_model, fitter))

    print("\n" + "="*90)
    print(f"{'Strategy':<10} {'Inliers':<10} {'Outliers':<10} {'Params (ax, ay, az, e1, e2)'}")
    print("-" * 90)
    for row in results_table:
        print(f"{row[0]:<10} {row[1]:<10} {row[2]:<10} {row[3]}")
    print("="*90)

    # --- COMPUTATION PHASE (Restore missing logic) ---
    occupied_mesh = None
    regenerated_mesh = None
    
    if viz_models:
        name, model, fit_obj = viz_models[-1]
        
        # 1. Surface Occupancy
        print("\n=== Computing Surface Occupancy (1cm Resolution) ===")
        mu = model.radial_distance_approximation(fit_obj.points)
        dists = np.linalg.norm(fit_obj.points - mu, axis=1)
        inlier_mask = dists <= 0.10
        points_in_shell = fit_obj.points[inlier_mask]
        
        occupied_mesh = utils.get_occupied_mesh(model, points_in_shell, pixel_size=0.01, min_neighbors=5)
        occupied_mesh.paint_uniform_color([1, 0.7, 0]) # Gold
        occupied_mesh.compute_vertex_normals()
        occupied_mesh.rotate(fit_obj.R_init, center=(0,0,0))
        occupied_mesh.translate(fit_obj.center)
        
        # 2. Transmission / Regeneration (Deviation Map)
        print("\n=== Simulating Transmission (Geometry Image) ===")
        
        print("Reconstructing Superquadric Surface Point Cloud...")
        # New API
        regenerated_pcd = utils.get_model_cloud(model, width=360, height=180)
        
        # Color cyan
        regenerated_pcd.paint_uniform_color([0, 1, 1])
        
        # Transform
        regenerated_pcd.rotate(fit_obj.R_init, center=(0,0,0))
        regenerated_pcd.translate(fit_obj.center)
    
    # --- VISUALIZATION AGGREGATION ---
    print("\n=== Visualization Phase: Simultaneous Windows ===")
    
    # Prepare Data for Window 1 (Original)
    p1_points = np.asarray(pcd.points)
    p1_colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    
    # Prepare Data for Window 2 (SQ Fit)
    p2_mesh_data = None
    if viz_models:
        name, model, fit_obj = viz_models[-1]
        
        # Window 2: Green Mesh
        mesh_sq = utils.get_mesh(model, resolution=50)
        mesh_sq.rotate(fit_obj.R_init, center=(0,0,0))
        mesh_sq.translate(fit_obj.center)
        p2_mesh_data = (
            np.asarray(mesh_sq.vertices),
            np.asarray(mesh_sq.triangles),
            [0, 1, 0] # Green Mesh
        )
        
        # 3. Analyze Deviations (With Logic)
        print("Analyzing Deviations...")
        # Use absolute distance: 10cm threshold

        num_surf, num_dev, inlier_mask = utils.analyze_inliers(model, fit_obj.points, distance_threshold=0.1)
        print(f"Stats: Surface Points={num_surf}, Deviation Points={num_dev} (Distance < 10cm)")
        
        # Segregate Points
        # Note: fit_obj.points are CANONICAL. Transform to WORLD for display.
        pts_can = fit_obj.points
        pts_world = (pts_can @ fit_obj.R_init.T) + fit_obj.center
        
        pts_surface = pts_world[inlier_mask]
        pts_deviation = pts_world[~inlier_mask]
        
        # Save encoded data for decoder testing
        output_dir = os.path.join(os.path.dirname(__file__), '..', 'encoded_data')
        os.makedirs(output_dir, exist_ok=True)
        
        params_array = np.array([model.ax, model.ay, model.az, model.e1, model.e2])
        np.save(os.path.join(output_dir, 'sq_params.npy'), params_array)
        np.save(os.path.join(output_dir, 'sq_center.npy'), fit_obj.center)
        np.save(os.path.join(output_dir, 'sq_rotation.npy'), fit_obj.R_init)
        
        
        # --- NEW: Generate Detailed Results Report ---
        results_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
        os.makedirs(results_dir, exist_ok=True)
        output_file = os.path.join(results_dir, 'output.txt')
        
        # 1. Bounding Box Info (Oriented)
        # Note: fit_obj.bbox is the OBB of the aligned cloud used for initialization
        obb = pcd.get_oriented_bounding_box()
        obb_center = obb.center
        obb_extent = obb.extent
        
        # 2. PCA Info (Principal Components of Original Cloud)
        # We can re-compute or use open3d
        mean, cov = pcd.compute_mean_and_covariance()
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        # Sort descending
        idx = eigenvalues.argsort()[::-1]   
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:,idx]
        
        with open(output_file, 'w') as f:
            f.write("=== Superquadric Fitting Results ===\n\n")
            
            f.write("1. Superquadric Parameters:\n")
            f.write(f"   ax: {model.ax:.6f}\n")
            f.write(f"   ay: {model.ay:.6f}\n")
            f.write(f"   az: {model.az:.6f}\n")
            f.write(f"   e1: {model.e1:.6f}\n")
            f.write(f"   e2: {model.e2:.6f}\n")
            f.write("\n")
            
            f.write("2. Point Cloud Statistics:\n")
            f.write(f"   Total Points: {len(fit_obj.points)}\n")
            f.write(f"   Surface Inliers (<10cm): {num_surf}\n")
            f.write(f"   Deviation Outliers: {num_dev}\n")
            f.write("\n")
            
            f.write("3. Oriented Bounding Box (Original Cloud):\n")
            f.write(f"   Center: [{obb_center[0]:.6f}, {obb_center[1]:.6f}, {obb_center[2]:.6f}]\n")
            f.write(f"   Extent: [{obb_extent[0]:.6f}, {obb_extent[1]:.6f}, {obb_extent[2]:.6f}]\n")
            f.write("\n")
            
            f.write("4. PCA Analysis (Eigenvalues/Vectors):\n")
            f.write(f"   Eigenvalues: {eigenvalues}\n")
            f.write("   Eigenvectors (Columns):\n")
            f.write(f"{eigenvectors}\n")
            
        print(f"Results saved to: {output_file}")
        
        print(f"\nEncoded data saved to: {output_dir}")
        
        # --- NEW: Save Deviation Points to PLY for Deviation Reduction Script ---
        viz_output_dir = os.path.join(os.path.dirname(__file__), '..', 'viz_output')
        os.makedirs(viz_output_dir, exist_ok=True)
        dev_file_path = os.path.join(viz_output_dir, '05_Deviation_Points_>10cm.ply')
        
        pcd_dev = o3d.geometry.PointCloud()
        # Transform back to world frame? 
        # pts_deviation IS in world frame (pts_world = ... + center)
        pcd_dev.points = o3d.utility.Vector3dVector(pts_deviation)
        o3d.io.write_point_cloud(dev_file_path, pcd_dev)
        print(f"Saved deviation points to: {dev_file_path}")
        
        # Separate windows for Surface and Deviations
        p3_points = pts_surface  # Window 3: Surface Points (Cyan)
        p4_points = pts_deviation  # Window 4: Deviation Points (Red)


    # Launch Processes
    print("Launching Process 1: Original Rock...")
    proc1 = multiprocessing.Process(target=show_window_proc, args=("Window 1: Original Rock", p1_points, None, p1_colors))
    proc1.start()
    
    print("Launching Process 2: Superquadric Fit (Mesh)...")
    proc2 = multiprocessing.Process(target=show_window_proc, args=("Window 2: Superquadric Fit", None, p2_mesh_data, None))
    proc2.start()

    print("Launching Process 3: Surface Points (Cyan)...")
    # Cyan color for surface points
    p3_colors = np.tile([0, 1, 1], (len(p3_points), 1))
    proc3 = multiprocessing.Process(target=show_window_proc, args=("Window 3: Surface Points", p3_points, None, p3_colors))
    proc3.start()
    
    print("Launching Process 4: Deviation Points (Red)...")
    # Red color for deviation points
    p4_colors = np.tile([1, 0, 0], (len(p4_points), 1))
    proc4 = multiprocessing.Process(target=show_window_proc, args=("Window 4: Deviation Points", p4_points, None, p4_colors))
    proc4.start()
    
    print("All 4 windows opened. Close them to finish script.")
    proc1.join()
    proc2.join()
    proc3.join()
    proc4.join()


if __name__ == "__main__":
    main()
