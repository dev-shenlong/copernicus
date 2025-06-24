import open3d as o3d
import numpy as np
import logging
import argparse

class Converter:
    def __init__(self, input_file, output_file, alpha=0.6, voxel_size=0.05, radii=None, depth=10, debug=False):
        self.input_file = input_file
        self.output_file = output_file
        self.alpha = alpha
        self.voxel_size = voxel_size
        self.radii = radii if radii is not None else [0.005, 0.01, 0.02, 0.04]  # Multiple radii for ball pivoting
        self.depth = depth
        self.configure_logger()
        if debug:
            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        
        # Initialize attributes
        self.point_cloud = None
        self.down_point_cloud = None
        self.mesh = None
        self.densities = None  # For Poisson reconstruction
    
    @property
    def alpha(self):
        return self._alpha
    
    @alpha.setter
    def alpha(self, alpha):
        if alpha > 0:
            self._alpha = alpha
        else:
            raise ValueError("Error!! Received alpha value less than 0")
    
    @property
    def voxel_size(self):
        return self._voxel_size
    
    @voxel_size.setter
    def voxel_size(self, voxel_size):
        if voxel_size > 0:
            self._voxel_size = voxel_size
        else:
            raise ValueError("Error!! Received voxel_size value less than 0")
    
    def configure_logger(self):
        print('Configuring logger')
        logging.basicConfig(filename="pcd2ply.log",
                          format='%(asctime)s %(message)s',
                          filemode='w')
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
    
    def load_point_cloud(self):
        try:
            self.point_cloud = o3d.io.read_point_cloud(self.input_file)
            if len(self.point_cloud.points) == 0:
                raise ValueError("Point cloud is empty")
            self.logger.info(f"Successfully loaded point cloud with {len(self.point_cloud.points)} points")
            
            # Remove invalid points (NaN or infinite values)
            self.point_cloud.remove_non_finite_points()
            self.logger.info(f"After removing non-finite points: {len(self.point_cloud.points)} points")
            
            # Check if normals exist, estimate if not (needed for some reconstruction methods)
            if not self.point_cloud.has_normals():
                self.logger.info("Point cloud has no normals, estimating normals")
                self.point_cloud.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
                )
                
        except Exception as e:
            self.logger.critical(f"Critical Error! Unable to open point cloud file: {self.input_file}")
            self.logger.critical(f"Exception Caught! {e}")
            raise
    def save_mesh(self):
        if self.mesh is None or len(self.mesh.triangles) == 0:
            self.logger.error("Mesh is empty or None")
            raise ValueError("Cannot save empty mesh")
        else:
            self.logger.info(f"Saving mesh to {self.output_file}")
            success = o3d.io.write_triangle_mesh(self.output_file, self.mesh)
            if success:
                self.logger.info("Mesh saved successfully")
            else:
                self.logger.error("Failed to save mesh")
                raise RuntimeError("Failed to save mesh file")
        if self.mesh is None or len(self.mesh.triangles) == 0:
            self.logger.error("Mesh is empty or None")
            raise ValueError("Cannot save empty mesh")
        else:
            self.logger.info(f"Saving mesh to {self.output_file}")
            success = o3d.io.write_triangle_mesh(self.output_file, self.mesh)
            if success:
                self.logger.info("Mesh saved successfully")
            else:
                self.logger.error("Failed to save mesh")
                raise RuntimeError("Failed to save mesh file")
    
    def simplify_pcd(self):
        if self.point_cloud is None:
            raise ValueError("Point cloud not loaded. Call load_point_cloud() first.")
        
        self.logger.info(f"Downsampling the point cloud with voxel size: {self.voxel_size}")
        self.down_point_cloud = self.point_cloud.voxel_down_sample(voxel_size=self.voxel_size)
        self.logger.info(f"Point cloud downsampled from {len(self.point_cloud.points)} to {len(self.down_point_cloud.points)} points")
        
        # Remove any remaining invalid points after downsampling
        self.down_point_cloud.remove_non_finite_points()
        
        # Ensure normals are preserved/estimated for downsampled cloud
        if not self.down_point_cloud.has_normals():
            self.logger.info("Estimating normals for downsampled point cloud")
            self.down_point_cloud.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size*4, max_nn=30)
            )
        
        # Orient normals consistently (important for Poisson reconstruction)
        try:
            self.down_point_cloud.orient_normals_consistent_tangent_plane(k=15)
            self.logger.info("Oriented normals consistently")
        except Exception as e:
            self.logger.warning(f"Could not orient normals consistently: {e}")
            # Try alternative normal orientation
            try:
                self.down_point_cloud.orient_normals_to_align_with_direction()
                self.logger.info("Oriented normals using direction alignment")
            except:
                self.logger.warning("Normal orientation failed, proceeding with unoriented normals")
    
    def reconstruct_mesh(self, reconstruction_method="alpha"):
        if self.down_point_cloud is None:
            raise ValueError("Downsampled point cloud not available. Call simplify_pcd() first.")
        
        if reconstruction_method == 'alpha':
            self.logger.info(f"Reconstructing mesh with alpha shape, alpha = {self.alpha}")
            self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                self.down_point_cloud, self.alpha
            )
            
        elif reconstruction_method == 'ball_pivoting':  
            self.logger.info(f"Reconstructing mesh with ball pivoting, radii = {self.radii}")
            # Convert to numpy array if needed
            radii_array = o3d.utility.DoubleVector(self.radii)
            self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                self.down_point_cloud, radii_array
            )
            
        elif reconstruction_method == 'poisson':
            self.logger.info(f"Reconstructing mesh with Poisson reconstruction, depth = {self.depth}")
            
            # Use special preprocessing for Poisson
            processed_pcd = self.preprocess_for_poisson()
            
            # Validate point cloud before Poisson reconstruction
            if not processed_pcd.has_normals():
                raise ValueError("Point cloud must have normals for Poisson reconstruction")
            
            # Check for valid normals
            normals = np.asarray(processed_pcd.normals)
            valid_normals = ~(np.isnan(normals).any(axis=1) | np.isinf(normals).any(axis=1))
            if not np.all(valid_normals):
                self.logger.warning(f"Found {np.sum(~valid_normals)} invalid normals out of {len(normals)}")
                # Keep only points with valid normals
                processed_pcd = processed_pcd.select_by_index(np.where(valid_normals)[0])
                self.logger.info(f"After removing invalid normals: {len(processed_pcd.points)} points")
            
            # Ensure we have enough points for Poisson reconstruction
            if len(processed_pcd.points) < 100:
                raise ValueError("Not enough points for Poisson reconstruction (minimum 100 required)")
            
            try:
                # Poisson reconstruction with adjusted parameters
                self.mesh, self.densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                    processed_pcd, depth=self.depth, width=0, scale=1.1, linear_fit=False
                )
                
                # Check if mesh was created successfully
                if len(self.mesh.vertices) == 0 or len(self.mesh.triangles) == 0:
                    raise ValueError("Poisson reconstruction failed to generate mesh")
                
                # Optional: Remove low density vertices (noise reduction)
                if self.densities is not None and len(self.densities) > 0:
                    # Use a more conservative threshold for density filtering
                    density_threshold = np.quantile(self.densities, 0.05)  # Remove bottom 5%
                    vertices_to_remove = self.densities < density_threshold
                    self.mesh.remove_vertices_by_mask(vertices_to_remove)
                    self.logger.info(f"Removed {np.sum(vertices_to_remove)} low-density vertices from Poisson reconstruction")
                    
            except Exception as e:
                self.logger.error(f"Poisson reconstruction failed: {e}")
                # Fallback to alpha shape if Poisson fails
                self.logger.info("Falling back to alpha shape reconstruction")
                self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                    self.down_point_cloud, self.alpha
                )
        
        else:
            raise ValueError(f"Unsupported reconstruction method: {reconstruction_method}")
        
        if self.mesh is not None and len(self.mesh.triangles) > 0:
            self.logger.info(f"Mesh created with {len(self.mesh.triangles)} triangles and {len(self.mesh.vertices)} vertices")
            
            # Compute vertex normals if not already present
            if not self.mesh.has_vertex_normals():
                self.logger.info("Computing vertex normals")
                self.mesh.compute_vertex_normals()
                
            # Optional mesh cleaning
            self.mesh.remove_duplicated_triangles()
            self.mesh.remove_duplicated_vertices()
            self.mesh.remove_non_manifold_edges()
            self.logger.info("Applied mesh cleaning operations")
            
        else:
            self.logger.warning("Mesh reconstruction failed or resulted in empty mesh")
            self.mesh = None
    
    def preprocess_for_poisson(self):
        """Special preprocessing for Poisson reconstruction"""
        if self.down_point_cloud is None:
            raise ValueError("Downsampled point cloud not available.")
        
        # Create a copy to avoid modifying the original
        pcd_copy = self.down_point_cloud
        
        # Remove statistical outliers
        pcd_copy, _ = pcd_copy.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        self.logger.info(f"After outlier removal: {len(pcd_copy.points)} points")
        
        # Estimate normals with larger search radius for stability
        pcd_copy.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size*8, max_nn=50)
        )
        
        # Orient normals consistently
        try:
            pcd_copy.orient_normals_consistent_tangent_plane(k=20)
        except:
            try:
                pcd_copy.orient_normals_to_align_with_direction(orientation_reference=np.array([0., 0., 1.]))
            except:
                self.logger.warning("Could not orient normals, using as-is")
        
        return pcd_copy
        if self.mesh is None or len(self.mesh.triangles) == 0:
            self.logger.error("Mesh is empty or None")
            raise ValueError("Cannot save empty mesh")
        else:
            self.logger.info(f"Saving mesh to {self.output_file}")
            success = o3d.io.write_triangle_mesh(self.output_file, self.mesh)
            if success:
                self.logger.info("Mesh saved successfully")
            else:
                self.logger.error("Failed to save mesh")
                raise RuntimeError("Failed to save mesh file")
    
    def convert(self, reconstruction_method="alpha"):
        """Complete conversion pipeline"""
        try:
            self.load_point_cloud()
            self.simplify_pcd()
            self.reconstruct_mesh(reconstruction_method)
            self.save_mesh()
            print(f"Conversion completed successfully: {self.input_file} -> {self.output_file}")
            print(f"Method used: {reconstruction_method}")
            if self.mesh:
                print(f"Final mesh: {len(self.mesh.vertices)} vertices, {len(self.mesh.triangles)} triangles")
        except Exception as e:
            self.logger.error(f"Conversion failed: {e}")
            print(f"Conversion failed: {e}")
            raise
    
    def visualize_results(self):
        """Optional method to visualize the point cloud and resulting mesh"""
        if self.point_cloud is None:
            print("No point cloud to visualize")
            return
            
        geometries = []
        
        # Add original point cloud (in blue)
        if self.point_cloud is not None:
            pcd_vis = self.point_cloud.paint_uniform_color([0, 0, 1])  # Blue
            geometries.append(pcd_vis)
        
        # Add mesh (in red wireframe)
        if self.mesh is not None:
            mesh_vis = self.mesh.paint_uniform_color([1, 0, 0])  # Red
            geometries.append(mesh_vis)
        
        if geometries:
            o3d.visualization.draw_geometries(geometries, 
                                            window_name="Point Cloud to Mesh Conversion Results")

# Example usage with different reconstruction methods:

def main():
    parser = argparse.ArgumentParser(description="Convert PCD file to Mesh using different reconstruction methods.")
    parser.add_argument("-i", "--input", required=True, help="Path to the input PCD file (without extension)")
    parser.add_argument("--alpha", type=float, default=0.6, help="Alpha value for alpha shape reconstruction")
    parser.add_argument("--voxel", type=float, default=0.05, help="Voxel size for downsampling")
    parser.add_argument("--depth", type=int, default=9, help="Depth value for Poisson reconstruction")
    parser.add_argument("--debug", action='store_true', help="Enable debug mode")

    args = parser.parse_args()

    input_file = args.input

    try:
        # Alpha shape reconstruction
        converter_alpha = Converter(
            input_file=f"{input_file}.pcd",
            output_file=f"{input_file}_alpha.ply",
            alpha=args.alpha,
            voxel_size=args.voxel,
            debug=args.debug
        )
        converter_alpha.convert(reconstruction_method="alpha")

        # Ball pivoting reconstruction
        converter_bp = Converter(
            input_file=f"{input_file}.pcd",
            output_file=f"{input_file}_bp.ply",
            voxel_size=args.voxel,
            radii=[0.005, 0.01, 0.02, 0.04],
            debug=args.debug
        )
        converter_bp.convert(reconstruction_method="ball_pivoting")

        # Poisson reconstruction
        converter_poisson = Converter(
            input_file=f"{input_file}.pcd",
            output_file=f"{input_file}_poisson.ply",
            voxel_size=args.voxel,
            depth=args.depth,
            debug=args.debug
        )
        converter_poisson.convert(reconstruction_method="poisson")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()