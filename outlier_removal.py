import open3d as o3d
import numpy as np

# Function to load .xyz point cloud
def load_point_cloud(file_path):
    # Load the point cloud from .xyz file
    point_cloud = np.loadtxt(file_path, delimiter=' ')
    
    # Convert numpy array to open3d point cloud format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    return pcd

# Function to remove outliers from a point cloud
def remove_outliers(pcd, nb_neighbors=5, std_ratio=2):
    # Remove statistical outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    
    # Keep only inliers
    inlier_cloud = pcd.select_by_index(ind)
    
    return inlier_cloud, ind

# Function to save the cleaned point cloud to a .xyz file
def save_point_cloud(pcd, output_file):
    np.savetxt(output_file, np.asarray(pcd.points), fmt='%.6f', delimiter=' ')

# Function to visualize point clouds before and after outlier removal
def visualize_point_clouds(original_pcd, inlier_pcd):
    
    # Set color for the original point cloud (red)
    original_pcd.paint_uniform_color([1, 0, 0])  # Red color
    
    # Set color for the cleaned point cloud (green)
    inlier_pcd.paint_uniform_color([0, 0, 1])  # Green color
    
    # Visualize both point clouds side by side
    o3d.visualization.draw_geometries([original_pcd], window_name="Original Point Cloud")
    o3d.visualization.draw_geometries([inlier_pcd], window_name="Cleaned Point Cloud")

# Main function to load, remove outliers, visualize, and save the result
def process_point_cloud(input_file, output_file):
    # Load the point cloud
    pcd = load_point_cloud(input_file)
    
    # Remove outliers
    inlier_cloud, ind = remove_outliers(pcd)
    
    # Visualize the point cloud before and after outlier removal
    visualize_point_clouds(pcd, inlier_cloud)
    
    # Save the cleaned point cloud to a new file
    save_point_cloud(inlier_cloud, output_file)
    print(f"Outliers removed. Cleaned point cloud saved to {output_file}")

input_file = r"C:\.xyz"  # Replace with your .xyz file
output_file = r"C:\.xyz"
process_point_cloud(input_file, output_file)
