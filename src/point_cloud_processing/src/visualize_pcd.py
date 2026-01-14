import open3d as o3d
import os
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing

FILENAME_base = "base_cloud.ply"
FILENAME_changed = "Changed.ply"
class cluster_properties:
    def __init__(self):
        pass
        
    def get_centroid(self, cluster_pcd):
        """Returns the centroid of the cluster (x, y, z)."""
        return cluster_pcd.get_center()
        
    def get_axis_aligned_bbox(self, cluster_pcd):
        """Returns the axis aligned bounding box."""
        return cluster_pcd.get_axis_aligned_bounding_box()

    def get_dimensions(self, cluster_pcd):
        """Returns the dimensions (extent) of the axis aligned bounding box."""
        bbox = self.get_axis_aligned_bbox(cluster_pcd)
        return bbox.get_extent()
        
    def calculate_fractal_dimension(self, cluster_pcd):
        """
        Calculates the fractal dimension using the Box-counting method.
        FD = -slope(log(N(s)) vs log(s))
        """
        # Define scales (box sizes). 
        # We start small (detail) and go up.
        # Use logspace to get evenly distributed points in log-scale.
        # Heuristic: from 0.05m to max_dimension/2
        
        dims = self.get_dimensions(cluster_pcd)
        max_dim = max(dims)
        min_scale = 0.05 # 5cm resolution
        max_scale = max_dim / 2.0
        
        if min_scale >= max_scale:
             return 0.0 # Object too small for this analysis
             
        scales = np.logspace(np.log10(min_scale), np.log10(max_scale), num=5)
        counts = []
        
        for scale in scales:
            # helper: voxel_down_sample creates a point for every occupied voxel
            # so len(downsampled.points) is exactly N(s)
            downsampled = cluster_pcd.voxel_down_sample(voxel_size=scale)
            count = len(downsampled.points)
            if count > 0:
                counts.append(count)
            else:
                counts.append(1) # avoid log(0)

        # Fit line: log(N(s)) = -D * log(s) + C
        # y = mx + c
        # x = log(scales)
        # y = log(counts)
        # m should be negative. Fractal dimension D = -m
        
        coeffs = np.polyfit(np.log(scales), np.log(counts), 1)
        fractal_dimension = -coeffs[0]
        
        return fractal_dimension

    def calculate_pca(self, cluster_pcd):
        """
        Calculates PCA eigenvalues and eigenvectors for the cluster.
        Returns:
            dict: {"eigenvalues": [e1, e2, e3], "eigenvectors": [[v1], [v2], [v3]]}
        """
        mean, covariance = cluster_pcd.compute_mean_and_covariance()
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        # Sort by eigenvalue (descending)
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        return {
            "eigenvalues": eigenvalues,
            "eigenvectors": eigenvectors
        }
        
    def compute_properties(self, cluster_pcd):
        """Computes and returns a dictionary of all cluster properties."""
        pca_data = self.calculate_pca(cluster_pcd)
        return {
            "centroid": self.get_centroid(cluster_pcd),
            "bbox": self.get_axis_aligned_bbox(cluster_pcd),
            "dimensions": self.get_dimensions(cluster_pcd),
            "fractal_dimension": self.calculate_fractal_dimension(cluster_pcd),
            "eigenvalues": pca_data["eigenvalues"],
            "eigenvectors": pca_data["eigenvectors"]
        }
        
    def generate_cluster_entry(self, cluster_id, cluster_pcd):
        """Generates a database entry for a single cluster."""
        props = self.compute_properties(cluster_pcd)
        return {
            "id": cluster_id,
            "points": cluster_pcd,
            "num_points": len(cluster_pcd.points),
            "centroid": props["centroid"],
            "bbox": props["bbox"],
            "dimensions": props["dimensions"],
            "fractal_dimension": props["fractal_dimension"],
            "eigenvalues": props["eigenvalues"],
            "eigenvectors": props["eigenvectors"]
        }


class cluster_filtering:
    def __init__(self):
        pass

    def remove_ground_plane(self, pcd, distance_threshold=0.2, ransac_n=3, num_iterations=1000, eps=0.05, min_points=10):

        print(f"Removing ground plane with threshold={distance_threshold}, n={ransac_n}, iterations={num_iterations}")
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                               ransac_n=ransac_n,
                                               num_iterations=num_iterations)
        
        non_ground_cloud = pcd.select_by_index(inliers, invert=True)
        
        # Call clustering on the non-ground points
        return self.perform_clustering(non_ground_cloud, eps=eps, min_points=min_points)

    def perform_clustering(self, pcd, eps=0.07, min_points=10):

        print(f"Clustering with eps={eps}, min_points={min_points}...")
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

        max_label = labels.max()
        print(f"Point cloud has {max_label + 1} clusters")
        
        # Calculate cluster counts first to find the maximum
        cluster_counts = {}
        for i in range(max_label + 1):
            cluster_counts[i] = (labels == i).sum()
            
        if not cluster_counts:
            print("No clusters found.")
            return o3d.geometry.PointCloud(), []
            
        max_points = max(cluster_counts.values())
        print(f"Max cluster size: {max_points} points")
        
        threshold_ratio = 0.08
        min_size_threshold = max_points * threshold_ratio
        print(f"Filtering with relative threshold {threshold_ratio*100}% (Keep >= {int(min_size_threshold)} points)")
        
        valid_indices = []
        valid_clusters = []
        
        for i in range(max_label + 1):
            count = cluster_counts[i]
            # print(f"Cluster {i}: {count} points", end="") 
            if count >= min_size_threshold:
                print(f"Cluster {i}: {count} points [KEPT]")
                
                # Get indices for this cluster
                cluster_indices = np.where(labels == i)[0]
                valid_indices.extend(cluster_indices)
                
                # Extract Cluster Point Cloud
                cluster_pcd = pcd.select_by_index(cluster_indices)
                valid_clusters.append(cluster_pcd)
                
        if not valid_indices:
            print("No clusters passed the threshold.")
            return o3d.geometry.PointCloud(), []

        print(f"Keeping {len(valid_indices)} points from {max_label + 1} clusters.")
        
        filtered_pcd = pcd.select_by_index(valid_indices)
        
        # filtered_labels = labels[valid_indices]
        
        # Color the clusters - COMMENTED OUT TO PRESERVE ORIGINAL RGB
        # colors = plt.get_cmap("tab20")(filtered_labels / (max_label if max_label > 0 else 1))
        # colors[filtered_labels < 0] = 0  # set noise to black
        # filtered_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        
        return filtered_pcd, valid_clusters

def process_cloud(filename, title_prefix):
    pcd_dir = os.path.join(os.path.dirname(__file__), '..', 'point_clouds')
    file_path = os.path.join(pcd_dir, filename)

    if not os.path.exists(file_path):
        print(f"Error: File not found at {file_path}")
        return None, None

    print(f"\nProcessing: {filename}")
    pcd = o3d.io.read_point_cloud(file_path)
    
    if pcd.is_empty():
        print("Warning: Point cloud is empty or failed to load.")
        return None, None

    print(f"Successfully loaded {pcd}")
    
    filtering_processor = cluster_filtering()
    properties_processor = cluster_properties()
    
    # Process the point cloud (Ground Removal -> Clustering)
    filtered_objects, clusters = filtering_processor.remove_ground_plane(pcd, distance_threshold=0.2, eps=0.05, min_points=10)
    
    # Generate Cluster DB
    cluster_db = []
    print(f"\n--- Cluster Properties ({filename}) ---")
    for i, cluster in enumerate(clusters):
        entry = properties_processor.generate_cluster_entry(i, cluster)
        cluster_db.append(entry)
        
        center = entry["centroid"]
        dims = entry["dimensions"]
        evals = entry["eigenvalues"]
        evecs = entry["eigenvectors"]
        
        print(f"Cluster ID: {entry['id']}")
        print(f"   -> Points: {entry['num_points']}")
        print(f"   -> Centroid: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
        print(f"   -> Dimensions: [{dims[0]:.3f}, {dims[1]:.3f}, {dims[2]:.3f}]")
        print(f"   -> Fractal Dim: {entry['fractal_dimension']:.3f}")
        print(f"   -> PCA Eigenvalues: [{evals[0]:.3f}, {evals[1]:.3f}, {evals[2]:.3f}]")
        print(f"   -> PCA Eigenvectors:")
        print(f"      v1: [{evecs[0][0]:.3f}, {evecs[1][0]:.3f}, {evecs[2][0]:.3f}]")
        print(f"      v2: [{evecs[0][1]:.3f}, {evecs[1][1]:.3f}, {evecs[2][1]:.3f}]")
        print(f"      v3: [{evecs[0][2]:.3f}, {evecs[1][2]:.3f}, {evecs[2][2]:.3f}]")
        print("--------------------------")
        
    return filtered_objects, cluster_db

def compare_clusters(c1, c2, threshold=1.5):
    """
    Compares two clusters based on their properties.
    Returns True if they are considered the same object.
    
    Parameters to compare:
    - Centroid (Euclidean dist)
    - Dimensions (Euclidean dist)
    - Eigenvalues (Euclidean dist)
    - Eigenvectors (Euclidean dist of matched vectors, handling sign ambiguity)
    """
    # 1. Centroid
    dist_centroid = np.linalg.norm(c1["centroid"] - c2["centroid"])
    if dist_centroid > threshold:
        return False
        
    # 2. Dimensions
    dist_dims = np.linalg.norm(c1["dimensions"] - c2["dimensions"])
    if dist_dims > threshold:
        return False
        
    # 3. Eigenvalues
    dist_evals = np.linalg.norm(c1["eigenvalues"] - c2["eigenvalues"])
    if dist_evals > threshold:
        return False
        
    # 4. Eigenvectors
    # Sum of distances between corresponding eigenvectors
    # Check both v and -v to handle sign ambiguity
    dist_evecs = 0
    for i in range(3):
        v1 = c1["eigenvectors"][:, i]
        v2 = c2["eigenvectors"][:, i]
        
        d_pos = np.linalg.norm(v1 - v2)
        d_neg = np.linalg.norm(v1 + v2)
        dist_evecs += min(d_pos, d_neg)
        
    if dist_evecs > threshold:
        return False
        
    return True



def create_grid(size=20, n=20):
    """Creates a simple grid on the XY plane."""
    lines = []
    points = []
    line_color = [0.8, 0.8, 0.8] # Light gray
    colors = []

    # Create points and lines
    step = size / n
    start = -size / 2
    
    # Grid lines along X
    for i in range(n + 1):
        y = start + i * step
        points.append([start, y, 0])
        points.append([start + size, y, 0])
        lines.append([len(points)-2, len(points)-1])
        colors.append(line_color)
        
    # Grid lines along Y
    for i in range(n + 1):
        x = start + i * step
        points.append([x, start, 0])
        points.append([x, start + size, 0])
        lines.append([len(points)-2, len(points)-1])
        colors.append(line_color)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set

def visualize_worker(points, colors, window_title):
    """Worker function to run visualization in a separate process."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Create coordinate frame and grid
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    grid = create_grid(size=30, n=30) # 30m grid
    
    print(f"Opening visualizer for {window_title}...")
    o3d.visualization.draw_geometries([pcd, coord_frame, grid], window_name=f"Open3D - {window_title}")

def main():
    files = [(FILENAME_base, "Base Cloud"), (FILENAME_changed, "Changed Cloud")]
    
    viz_data = []
    database_results = {} # store db for each file
    
    # Process all files first
    for filename, window_title in files:
        filtered_pcd, db = process_cloud(filename, window_title)
        
        database_results[filename] = db
        
        if filtered_pcd:
            # Extract data for multiprocessing (Open3D objects are not picklable)
            points = np.asarray(filtered_pcd.points)
            colors = np.asarray(filtered_pcd.colors)
            viz_data.append((points, colors, window_title))
        else:
            print(f"Failed to process {filename}")

    # --- Change Detection Logic ---
    base_db = database_results.get(FILENAME_base, [])
    changed_db = database_results.get(FILENAME_changed, [])
    
    new_objects_pcd = o3d.geometry.PointCloud()
    new_points = []
    new_colors = []
    
    print("\n--- Change Detection Analysis ---")
    
    for c_changed in changed_db:
        match_found = False
        for c_base in base_db:
            if compare_clusters(c_changed, c_base, threshold=1.5):
                match_found = True
                print(f"Match Found: Changed Cluster {c_changed['id']} == Base Cluster {c_base['id']}")
                break
        
        if not match_found:
            print(f"NEW OBJECT DETECTED: Changed Cluster {c_changed['id']}")
            # Add to new objects point cloud
            pts = np.asarray(c_changed["points"].points)
            clrs = np.asarray(c_changed["points"].colors)
            
            # If colors are empty (sometimes DBSCAN result), assign a distinct color (Red)
            if len(clrs) == 0:
                 clrs = np.tile([1, 0, 0], (len(pts), 1))
            
            if len(new_points) == 0:
                new_points = pts
                new_colors = clrs
            else:
                new_points = np.vstack((new_points, pts))
                new_colors = np.vstack((new_colors, clrs))

    if len(new_points) > 0:
        new_objects_pcd.points = o3d.utility.Vector3dVector(new_points)
        new_objects_pcd.colors = o3d.utility.Vector3dVector(new_colors)
        
        # Save to file
        output_filename = "scene_delta.ply"
        pcd_dir = os.path.join(os.path.dirname(__file__), '..', 'point_clouds')
        output_path = os.path.join(pcd_dir, output_filename)
        o3d.io.write_point_cloud(output_path, new_objects_pcd)
        print(f"Saved new objects to: {output_path}")
        
        viz_data.append((new_points, new_colors, "New Objects Detected"))
    else:
        print("No new objects detected.")

    # Start visualization processes
    processes = []
    print("\nStarting parallel visualizers...")
    for points, colors, title in viz_data:
        p = multiprocessing.Process(target=visualize_worker, args=(points, colors, title))
        p.start()
        processes.append(p)
        
    # Wait for all processes to finish
    for p in processes:
        p.join()

if __name__ == "__main__":
    main()
