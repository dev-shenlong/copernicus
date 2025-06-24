import open3d as o3d
import numpy as np


o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

print("Loading point cloud...")
pcd = o3d.io.read_point_cloud("1st_floor.pcd")
print(f"Loaded point cloud with {len(pcd.points)} points")

print("Downsampling point cloud...")
down_size_pcd = pcd.voxel_down_sample(voxel_size=0.05)
print(f"Downsampled to {len(down_size_pcd.points)} points")

alpha = 0.03
print(f"Creating alpha shape with alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
print(f"Created mesh with {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles")

print("Computing vertex normals...")
mesh.compute_vertex_normals()

print("Displaying mesh...")
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)