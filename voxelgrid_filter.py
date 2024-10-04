import open3d as o3d
import numpy as np
#Load xyz file
def load_xyz_file(file_path):

    data = np.loadtxt(file_path, delimiter=' ')
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(data[:, :3])
    return point_cloud
#Function for point cloud saving
def save_xyz_file(point_cloud, file_path):
    np.savetxt(file_path, np.asarray(point_cloud.points), delimiter=' ')

def downsample_point_cloud(point_cloud, voxel_size):
    downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size=voxel_size)
    return downsampled_point_cloud

def visualize_point_cloud(point_cloud, title="Point Cloud"):
    print(f"Visualizing: {title}")
    o3d.visualization.draw_geometries([point_cloud])

def main(input_xyz_file, output_xyz_file, voxel_size):
    pcd = load_xyz_file(input_xyz_file)
    visualize_point_cloud(pcd, title="Original Point Cloud")

    downsampled_pcd = downsample_point_cloud(pcd, voxel_size)

    visualize_point_cloud(downsampled_pcd, title="Downsampled Point Cloud")
    # Save the downsampled point cloud to an XYZ file
    save_xyz_file(downsampled_pcd, output_xyz_file)

if __name__ == "__main__":
    input_xyz_file = r"C:\.xyz"  # Path to your input XYZ file
    output_xyz_file = r"C:\.xyz"  # Path to save the output XYZ file
    voxel_size = 2  # Adjust this value based on your desired density
    main(input_xyz_file, output_xyz_file, voxel_size)












