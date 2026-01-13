import open3d as o3d
import os
import numpy as np
import matplotlib.pyplot as plt

#FILENAME = "Changed.ply"
FILENAME = "base_cloud.ply"
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
        
    def compute_properties(self, cluster_pcd):
        """Computes and returns a dictionary of all cluster properties."""
        return {
            "centroid": self.get_centroid(cluster_pcd),
            "bbox": self.get_axis_aligned_bbox(cluster_pcd),
            "dimensions": self.get_dimensions(cluster_pcd)
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
            "dimensions": props["dimensions"]
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
            print(f"Cluster {i}: {count} points", end="")
            if count >= min_size_threshold:
                print(" [KEPT]")
                
                # Get indices for this cluster
                cluster_indices = np.where(labels == i)[0]
                valid_indices.extend(cluster_indices)
                
                # Extract Cluster Point Cloud
                cluster_pcd = pcd.select_by_index(cluster_indices)
                valid_clusters.append(cluster_pcd)
                
            else:
                print(" [REMOVED]")
                
        if not valid_indices:
            print("No clusters passed the threshold.")
            return o3d.geometry.PointCloud(), []

        print(f"Keeping {len(valid_indices)} points from {max_label + 1} clusters.")
        
        filtered_pcd = pcd.select_by_index(valid_indices)
        
        filtered_labels = labels[valid_indices]
        
        # Color the clusters
        colors = plt.get_cmap("tab20")(filtered_labels / (max_label if max_label > 0 else 1))
        colors[filtered_labels < 0] = 0  # set noise to black
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        
        return filtered_pcd, valid_clusters

def main():
    pcd_dir = os.path.join(os.path.dirname(__file__), '..', 'point_clouds')
    file_path = os.path.join(pcd_dir, FILENAME)

    if not os.path.exists(file_path):
        print(f"Error: File not found at {file_path}")
        print(f"Please make sure '{FILENAME}' is in the {pcd_dir} directory.")
        return

    print(f"Loading point cloud from: {file_path}")
    pcd = o3d.io.read_point_cloud(file_path)
    
    if pcd.is_empty():
        print("Warning: Point cloud is empty or failed to load.")
        return

    print(f"Successfully loaded {pcd}")
    
    filtering_processor = cluster_filtering()
    properties_processor = cluster_properties()
    
    # Process the point cloud (Ground Removal -> Clustering)
    # The remove_ground_plane function now handles the full pipeline
    filtered_objects, clusters = filtering_processor.remove_ground_plane(pcd, distance_threshold=0.2, eps=0.05, min_points=10)
    
    # Generate Cluster DB and Print Properties
    cluster_db = []
    print("\n--- Cluster Properties ---")
    for i, cluster in enumerate(clusters):
        entry = properties_processor.generate_cluster_entry(i, cluster)
        cluster_db.append(entry)
        
        center = entry["centroid"]
        dims = entry["dimensions"]
        print(f"Cluster ID: {entry['id']}")
        print(f"   -> Points: {entry['num_points']}")
        print(f"   -> Centroid: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
        print(f"   -> Dimensions: [{dims[0]:.3f}, {dims[1]:.3f}, {dims[2]:.3f}]")
        print("--------------------------")
    
    print("Opening visualizer (Press 'q' to close, 'h' for help)...")
    o3d.visualization.draw_geometries([filtered_objects], window_name=f"Open3D - Clustering - {FILENAME}")

if __name__ == "__main__":
    main()
