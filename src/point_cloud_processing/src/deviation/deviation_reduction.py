import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt

class dev_red:
    def __init__(self):
        self.base_dir = os.path.dirname(__file__)
        self.viz_dir = os.path.join(self.base_dir, '../viz_output')
        self.input_filename = "05_Deviation_Points_>10cm.ply"
        
    def dbscan(self, pcd, eps=0.05, min_points=10):
        """
        Clusters the point cloud using DBSCAN.
        """
        print(f"Running DBSCAN (eps={eps}, min_points={min_points})...")
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
        return labels

    def subdivide_cluster(self, points, target_size=2000):
        """
        Subdivides a large cluster into smaller patches using K-Means.
        k = ceil(num_points / target_size)
        """
        from sklearn.cluster import KMeans
        
        n_points = len(points)
        if n_points <= target_size:
            return np.zeros(n_points, dtype=int)
            
        k = int(np.ceil(n_points / target_size))
        # print(f"  -> Splitting cluster of {n_points} points into {k} sub-clusters...")
        
        kmeans = KMeans(n_clusters=k, n_init=5, random_state=42)
        sub_labels = kmeans.fit_predict(points)
        return sub_labels

    def execute(self):
        # 1. Load Data
        filepath = os.path.join(self.viz_dir, self.input_filename)
        if not os.path.exists(filepath):
            print(f"Error: {filepath} not found.")
            return
            
        print(f"Loading {filepath}...")
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)
        
        if len(points) == 0:
            print("Point cloud is empty.")
            return
            
        # 2. Initial Clustering (DBSCAN) - Find "Islands"
        print("--- Step 1: Isolating Islands (DBSCAN) ---")
        base_labels = self.dbscan(pcd)
        max_base_label = base_labels.max()
        print(f"Found {max_base_label + 1} distinct islands.")
        
        # 3. Sub-Clustering (K-Means) - Break big islands into patches
        print("\n--- Step 2: Sub-dividing Large Islands (K-Means) ---")
        final_labels = np.full(len(points), -1)
        current_label_id = 0
        
        # Mapping for colors later
        final_to_base_map = {} 
        
        # Process Noise (-1) - Keep as -1
        final_labels[base_labels == -1] = -1
        
        for i in range(max_base_label + 1):
            mask = (base_labels == i)
            cluster_points = points[mask]
            
            # Subdivide
            sub_labels = self.subdivide_cluster(cluster_points, target_size=5000) # User wants smaller areas
            
            # Assign global IDs
            unique_subs = np.unique(sub_labels)
            for sub_id in unique_subs:
                sub_mask = (sub_labels == sub_id)
                
                # Assign new global ID
                global_mask = np.zeros(len(points), dtype=bool)
                # We need to map back to global indices.
                # Easiest way: Iterate mask indices
                global_indices = np.where(mask)[0][sub_mask]
                final_labels[global_indices] = current_label_id
                
                final_to_base_map[current_label_id] = i
                current_label_id += 1
        
        num_final_clusters = current_label_id
        print(f"Total Patches after subdivision: {num_final_clusters}")
        
        # Get unique patch IDs
        unique_final, counts_final = np.unique(final_labels, return_counts=True)
        valid_mask = (unique_final != -1)
        unique_final = unique_final[valid_mask]
        counts_final = counts_final[valid_mask]
        
        # Import geometry fitting
        import sys
        sys.path.append(os.path.join(os.path.dirname(__file__), '../utils'))
        import geometry_fitting as gf
        
        # 4. Batch Process All Patches
        print(f"\n=== Batch Processing All {len(unique_final)} Patches ===")
        
        # Load Global SQ Parameters
        encoded_dir = os.path.join(self.base_dir, '../encoded_data')
        sys.path.append(os.path.join(os.path.dirname(__file__), '../superquadrics'))
        from ems_core import Superquadric
        
        sq_params = np.load(os.path.join(encoded_dir, 'sq_params.npy'))
        sq_center = np.load(os.path.join(encoded_dir, 'sq_center.npy'))
        sq_rotation = np.load(os.path.join(encoded_dir, 'sq_rotation.npy'))
        global_sq = Superquadric(sq_params)
        
        # Results storage
        results = []
        
        for patch_id in unique_final:
            patch_mask = (final_labels == patch_id)
            patch_points = points[patch_mask]
            num_points = len(patch_points)
            
            print(f"\nProcessing Patch {patch_id}: {num_points} points")
            
            try:
                # PCA alignment
                R_pca, center_pca = gf.compute_pca_frame(patch_points)
                pts_local_canonical = (patch_points - center_pca) @ R_pca
                
                # Transform to global canonical for ray filtering
                pts_global_canonical = (patch_points - sq_center) @ sq_rotation
                
                # Ray-based filtering
                patch_centroid_global_canonical = (center_pca - sq_center) @ sq_rotation
                ray_direction = patch_centroid_global_canonical / (np.linalg.norm(patch_centroid_global_canonical) + 1e-12)
                
                point_norms = np.linalg.norm(pts_global_canonical, axis=1) + 1e-12
                cos_angles = np.dot(pts_global_canonical, ray_direction) / point_norms
                
                angle_threshold_deg = 30.0
                cos_threshold = np.cos(np.radians(angle_threshold_deg))
                ray_mask = cos_angles >= cos_threshold
                
                ray_filtered_points_local = pts_local_canonical[ray_mask]
                num_ray_filtered = len(ray_filtered_points_local)
                
                # EM Paraboloid Fitting
                if num_ray_filtered >= 6:
                    paraboloid_params, inlier_mask_em, num_inliers = gf.fit_paraboloid_em(
                        ray_filtered_points_local,
                        distance_threshold=0.05,
                        w_o_prior=0.3,
                        max_iters=20,
                        verbose=False
                    )
                    
                    a, b, c, d, e, f = paraboloid_params
                    num_outliers = num_ray_filtered - num_inliers
                    
                    results.append({
                        'patch_id': patch_id,
                        'total_points': num_points,
                        'ray_filtered_points': num_ray_filtered,
                        'paraboloid_inliers': num_inliers,
                        'paraboloid_outliers': num_outliers,
                        'inlier_percentage': 100.0 * num_inliers / num_ray_filtered,
                        'param_a': a,
                        'param_b': b,
                        'param_c': c,
                        'param_d': d,
                        'param_e': e,
                        'param_f': f
                    })
                    
                    print(f"  Success: {num_inliers}/{num_ray_filtered} inliers ({100*num_inliers/num_ray_filtered:.1f}%)")
                else:
                    print(f"  Skipped: Not enough points ({num_ray_filtered})")
                    results.append({
                        'patch_id': patch_id,
                        'total_points': num_points,
                        'ray_filtered_points': num_ray_filtered,
                        'paraboloid_inliers': 0,
                        'paraboloid_outliers': 0,
                        'inlier_percentage': 0.0,
                        'param_a': np.nan,
                        'param_b': np.nan,
                        'param_c': np.nan,
                        'param_d': np.nan,
                        'param_e': np.nan,
                        'param_f': np.nan
                    })
                    
            except Exception as e:
                print(f"  Error: {e}")
                results.append({
                    'patch_id': patch_id,
                    'total_points': num_points,
                    'ray_filtered_points': 0,
                    'paraboloid_inliers': 0,
                    'paraboloid_outliers': 0,
                    'inlier_percentage': 0.0,
                    'param_a': np.nan,
                    'param_b': np.nan,
                    'param_c': np.nan,
                    'param_d': np.nan,
                    'param_e': np.nan,
                    'param_f': np.nan
                })
        
        # Save to CSV
        import pandas as pd
        df = pd.DataFrame(results)
        csv_path = os.path.join(self.viz_dir, 'paraboloid_fitting_results.csv')
        df.to_csv(csv_path, index=False)
        
        print(f"\n=== Results saved to {csv_path} ===")
        print(f"Processed {len(results)} patches")
        print(f"Successful fits: {df['paraboloid_inliers'].gt(0).sum()}")
        
        # 4. Find Largest Patch
        unique_final, counts_final = np.unique(final_labels, return_counts=True)
        # Filter out noise (-1)
        valid_mask = (unique_final != -1)
        unique_final = unique_final[valid_mask]
        counts_final = counts_final[valid_mask]
        
        if len(unique_final) == 0:
            print("No valid clusters found.")
            return
            
        largest_cluster_id = unique_final[np.argmax(counts_final)]
        largest_size = np.max(counts_final)
        
        print(f"\nLargest Patch ID: {largest_cluster_id} with {largest_size} points")
        
        # 5. Visualize ONLY the Largest Patch (Canonical Frame)
        # We want to see the patch in its own "Principal Axis Frame" (Aligned with PCA).
        # We will center it at the Centroid (0,0,0) and align axes.

        largest_mask = (final_labels == largest_cluster_id)
        largest_patch_points = points[largest_mask]

        import sys
        sys.path.append(os.path.join(os.path.dirname(__file__), '../utils'))
        import geometry_fitting as gf
        
        print("\n--- Canonical Frame Analysis ---")
        
        # 1. Centroid & PCA Alignment
        #    We need to compute the PCA frame to align the patch.
        R_pca, center_pca = gf.compute_pca_frame(largest_patch_points)
        print(f"  Centroid: {center_pca}")
        
        # 2. Transform to Local Canonical Frame (Centered at Origin)
        #    P_local = (P_global - Centroid) @ R_pca
        pts_local_canonical = (largest_patch_points - center_pca) @ R_pca
        
        # 3. Visualization Objects
        geometries_to_draw = []

        # A. The Patch Itself (Green)
        pcd_canonical = o3d.geometry.PointCloud()
        pcd_canonical.points = o3d.utility.Vector3dVector(pts_local_canonical)
        pcd_canonical.paint_uniform_color([0, 1, 0]) # Green
        geometries_to_draw.append(pcd_canonical)
        
        # B. Coordinate Frame (at Origin)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
        geometries_to_draw.append(frame)
        
        # --- NEW: Global Superquadric Max Deviation Analysis ---
        # 1. Load Global SQ Parameters
        encoded_dir = os.path.join(self.base_dir, '../encoded_data')
        try:
            sys.path.append(os.path.join(os.path.dirname(__file__), '../superquadrics'))
            from ems_core import Superquadric
            
            sq_params = np.load(os.path.join(encoded_dir, 'sq_params.npy'))
            sq_center = np.load(os.path.join(encoded_dir, 'sq_center.npy'))
            sq_rotation = np.load(os.path.join(encoded_dir, 'sq_rotation.npy'))
            
            print("\n--- Global Superquadric Deviation Analysis ---")
            print(f"  Loaded Global SQ: {sq_params}")
            
            # 2. Transform Patch Points to Global Canonical Frame
            # P_local = (P_global - C_global) @ R_global
            # Note: We use the points from the original patch (in World Frame)
            pts_global_canonical = (largest_patch_points - sq_center) @ sq_rotation
            
            # 3. Compute Deviations
            global_sq = Superquadric(sq_params)
            
            # Project points to surface
            # radial_distance_approximation returns point on surface along the ray
            # But wait, radial approx assumes star-shape. SQ is star-shaped.
            mu = global_sq.radial_distance_approximation(pts_global_canonical)
            
            # Calculate distances (Euclidean Norm)
            # d = || P - mu ||
            dists = np.linalg.norm(pts_global_canonical - mu, axis=1)
            
            # 4. Ray-Based Filtering for Peak Finding
            # Construct ray from Global SQ Center (origin in canonical) through Patch Centroid
            # In global canonical frame, patch centroid is:
            patch_centroid_world = center_pca  # World frame
            patch_centroid_global_canonical = (patch_centroid_world - sq_center) @ sq_rotation
            
            # Ray direction: from origin to patch centroid
            ray_direction = patch_centroid_global_canonical / (np.linalg.norm(patch_centroid_global_canonical) + 1e-12)
            
            print(f"\n  Ray-Based Peak Finding:")
            print(f"    Patch Centroid (Global Canonical): {patch_centroid_global_canonical}")
            print(f"    Ray Direction: {ray_direction}")
            
            # Filter points close to this ray
            # Use angle threshold: cos(theta) > threshold (e.g., cos(30°) ≈ 0.866)
            angle_threshold_deg = 30.0
            cos_threshold = np.cos(np.radians(angle_threshold_deg))
            
            # For each point, compute angle with ray
            # cos(theta) = (point · ray) / |point|
            point_norms = np.linalg.norm(pts_global_canonical, axis=1) + 1e-12
            cos_angles = np.dot(pts_global_canonical, ray_direction) / point_norms
            
            ray_mask = cos_angles >= cos_threshold
            num_ray_points = np.sum(ray_mask)
            
            print(f"    Points within {angle_threshold_deg}° of ray: {num_ray_points}/{len(pts_global_canonical)}")
            
            if num_ray_points == 0:
                print("    WARNING: No points found along ray. Using all points.")
                ray_mask = np.ones(len(pts_global_canonical), dtype=bool)
            
            # Find max deviation among filtered points
            filtered_dists = dists[ray_mask]
            filtered_indices = np.where(ray_mask)[0]
            
            max_filtered_idx = np.argmax(filtered_dists)
            max_idx = filtered_indices[max_filtered_idx]
            max_dist = dists[max_idx]
            max_point_local = pts_global_canonical[max_idx]
            max_point_world = largest_patch_points[max_idx]
            
            print(f"  Maximum Deviation |d| (Ray-Filtered): {max_dist:.4f} m")
            print(f"  Max Point (World): {max_point_world}")
            
            # 5. Visualize "Peak Point" (Blue Sphere)
            # User requested to hide this
            # max_point_viz = pts_local_canonical[max_idx]
            # 
            # peak_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
            # peak_sphere.paint_uniform_color([0, 0, 1]) # Blue
            # peak_sphere.translate(max_point_viz)
            # geometries_to_draw.append(peak_sphere)
            # 
            # print("  -> Added Blue Sphere at max deviation point.")
            
            # 6. Paraboloid Fitting to Peak Region (EM-Based)
            print("\n  Fitting Paraboloid using EM Algorithm...")
            
            # Use all ray-filtered points for EM fitting
            ray_filtered_points_local = pts_local_canonical[ray_mask]
            
            print(f"    Input points for EM: {len(ray_filtered_points_local)}")
            
            if len(ray_filtered_points_local) >= 6:
                try:
                    # EM fitting with 5cm threshold
                    paraboloid_params, inlier_mask_em, num_inliers = gf.fit_paraboloid_em(
                        ray_filtered_points_local,
                        distance_threshold=0.05,
                        w_o_prior=0.3,
                        max_iters=20,
                        verbose=True
                    )
                    
                    a, b, c, d, e, f = paraboloid_params
                    print(f"\n    Final Paraboloid Equation:")
                    print(f"    z = {a:.4f}*x² + {b:.4f}*y² + {c:.4f}*xy")
                    print(f"        + {d:.4f}*x + {e:.4f}*y + {f:.4f}")
                    
                    # Get inlier points for visualization range
                    inlier_points_local = ray_filtered_points_local[inlier_mask_em]
                    
                    # Generate visualization points
                    x_min, x_max = np.min(inlier_points_local[:, 0]), np.max(inlier_points_local[:, 0])
                    y_min, y_max = np.min(inlier_points_local[:, 1]), np.max(inlier_points_local[:, 1])
                    
                    # Expand range slightly for better visualization
                    x_range = (x_min - 0.05, x_max + 0.05)
                    y_range = (y_min - 0.05, y_max + 0.05)
                    
                    paraboloid_pts = gf.generate_paraboloid_points(paraboloid_params, x_range, y_range, resolution=30)
                    
                    pcd_paraboloid = o3d.geometry.PointCloud()
                    pcd_paraboloid.points = o3d.utility.Vector3dVector(paraboloid_pts)
                    pcd_paraboloid.paint_uniform_color([0, 1, 1])  # Cyan
                    geometries_to_draw.append(pcd_paraboloid)
                    
                    print("    -> Added Cyan paraboloid visualization.")
                    
                except Exception as e:
                    print(f"    Paraboloid EM fitting failed: {e}")
                    import traceback
                    traceback.print_exc()
            else:
                print("    Not enough points for paraboloid fitting.")
            
            # 7. Generate Histogram
            
            # 6. Generate Histogram
            print("\n  Generating Deviation Histogram (0-2m, 0.1m bins)...")
            plt.figure(figsize=(10, 6))
            bins = np.arange(0, 2.1, 0.1)
            plt.hist(dists, bins=bins, edgecolor='black', alpha=0.7)
            plt.title('Histogram of Point Deviations from Superquadric Surface')
            plt.xlabel('Deviation Distance (m)')
            plt.ylabel('Number of Points')
            plt.xticks(bins)
            plt.grid(axis='y', alpha=0.75)
            
            # Save or Show? User likely wants to see it.
            # Showing it might block the Open3D window if not handled carefully.
            # But usually plt.show() blocks.
            # We can save it too.
            hist_path = os.path.join(self.viz_dir, 'deviation_histogram.png')
            plt.savefig(hist_path)
            print(f"  Histogram saved to {hist_path}")
            
            # Non-blocking show if possible, or show before/after Open3D
            plt.show(block=False) 
            
        except Exception as e:
            print(f"  WARNING: Could not perform Global SQ Analysis: {e}")
            import traceback
            traceback.print_exc()
            
        # --- End Global Analysis ---
        
        # --- NEW: Fit Superquadric (Patch Local) ---
        
        # --- NEW: Fit Superquadric (Patch Local) ---
        # User requested NOT to fit yet.
        # print("  Fitting Superquadric...")
        # try:
        #     params = gf.fit_superquadric(pts_local_canonical)
        #     a1, a2, a3, e1, e2 = params
        #     print(f"  Fitted SQ Params: a=({a1:.3f}, {a2:.3f}, {a3:.3f}), e=({e1:.3f}, {e2:.3f})")
        #     
        #     # Generate points for visualization
        #     pts_sq = gf.generate_superquadric_points(params, resolution=60)
        #     
        #     # Create PointCloud (Red)
        #     pcd_sq = o3d.geometry.PointCloud()
        #     pcd_sq.points = o3d.utility.Vector3dVector(pts_sq)
        #     pcd_sq.paint_uniform_color([1, 0, 0]) # Red
        #     geometries_to_draw.append(pcd_sq)
        #     
        # except Exception as e:
        #     print(f"  Fitting Failed: {e}")
            
        # --- End Fit ---
        
        # C. Ground Grid (for context)
        max_bound = np.max(np.abs(pts_local_canonical))
        grid_size = max_bound * 1.5
        grid = self.create_grid(size=grid_size, step=grid_size/10)
        geometries_to_draw.append(grid)
        
        print(f"Displaying Canonical View (Centroid at Origin, Aligned with Eigenvectors)...")
        print("  - Green: Point Cloud Patch")
        print("  - RGB Arrows: Principal Axes (X=Red, Y=Green, Z=Blue)")
        print("  - Gray Grid: Local XY Plane")
        
        o3d.visualization.draw_geometries(geometries_to_draw, window_name="Canonical Axis View")

    def create_grid(self, size=1.0, step=0.1):
        lines = []
        # Lines parallel to X-axis
        for y in np.arange(-size, size + step, step):
            lines.append([[ -size, y, 0], [ size, y, 0]])
        # Lines parallel to Y-axis
        for x in np.arange(-size, size + step, step):
            lines.append([[ x, -size, 0], [ x, size, 0]])
        
        line_set = o3d.geometry.LineSet()
        points = []
        edges = []
        for i, line in enumerate(lines):
            points.append(line[0])
            points.append(line[1])
            edges.append([2*i, 2*i+1])
        
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(edges)
        line_set.paint_uniform_color([0.5, 0.5, 0.5]) # Gray
        return line_set

def main():
    reducer = dev_red()
    reducer.execute()

if __name__ == "__main__":
    main()
