import numpy as np
from sklearn.decomposition import PCA
import open3d as o3d
from sksurgerysurfacematch.algorithms.goicp_registration import RigidRegistration
import pyvista as pv
pcd1 = o3d.io.read_point_cloud(r"C:\.xyz") #Path to digitalized point cloud
pcd2 = o3d.io.read_point_cloud(r"C:\.xyz") #Path to CAD model

points1 = np.asarray(pcd1.points)
points2 = np.asarray(pcd2.points)
print(np.shape(points1))
#Apply PCA
pca1 = PCA(n_components=3)
pca1.fit(points1)

pca2 = PCA(n_components=3)
pca2.fit(points2)
#Calculate rotation and translation
rotation = pca1.components_.T @ pca2.components_
translation = pca1.mean_ - rotation @ pca2.mean_

transformation_matrix1 = np.eye(4)
transformation_matrix1[:3, :3] = rotation
transformation_matrix1[:3, 3] = translation
print(transformation_matrix1)
points2_homogeneous = np.hstack((points2, np.ones((points2.shape[0], 1))))

points2_transformed_homogeneous = points2_homogeneous @ transformation_matrix1.T
points2_transformed = points2_transformed_homogeneous[:, :3]
#Visualize results after PCA
pcap1=pv.PolyData(points1)
pcap2=pv.PolyData(points2_transformed)
color1 = [0.5, 0, 0.5]
color2 = [0.3, 0, 0.4]

colors1 = np.full((points1.shape[0], 3), color2)
colors2 = np.full((points2_transformed.shape[0], 3), color1)

pcap1['Colors'] = colors1
pcap2['Colors'] = colors2
combined=pcap1+pcap2
combined.plot(background='black', point_size=3, show_axes=True, show_bounds=True)


##Go ICP algorithm
moving_cloud=points1
fixed_cloud=points2_transformed

registration = RigidRegistration(
    #rotation_limits=[-180,180],
    #num_moving_points=0,                     #Edit these parameters as needed
)


residual, transformation_matrix = registration.register(moving_cloud, fixed_cloud)

print("Transformation matrix:")
print(np.linalg.inv(transformation_matrix))

#Calculate final registration matrix
final_transfMatrix=np.linalg.inv(transformation_matrix)@transformation_matrix1
print('Final registration matrix: ',final_transfMatrix)

points = np.hstack((moving_cloud, np.ones((moving_cloud.shape[0], 1))))

transformed_points_h = np.dot(transformation_matrix, points.T).T

transformed_points = transformed_points_h[:, :3]
#Visualize final result
point_cloud1 = pv.PolyData(fixed_cloud)
point_cloud2 = pv.PolyData(transformed_points)

color1 = [0, 0.5, 0.5]
color2 = [0.3, 0, 0.4]

colors1 = np.full((fixed_cloud.shape[0], 3), color1)
colors2 = np.full((transformed_points.shape[0], 3), color2)
point_cloud1['Colors'] = colors1
point_cloud2['Colors'] = colors2
combined = point_cloud1 + point_cloud2

combined.plot(background='black', point_size=3, show_axes=True, show_bounds=True)