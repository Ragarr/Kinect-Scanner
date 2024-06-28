# Class that combines N Point Clouds into a single point cloud
import open3d as o3d
from typing import List, Union
import numpy as np

class Combiner:
    @staticmethod
    def combine_clouds(pcds: List[o3d.geometry.PointCloud],
                       voxel_size:float = 0.05) -> o3d.geometry.PointCloud:

        combined_pcd = pcds[0]
        for i in range(1, len(pcds)):
            # do global registration
            result_ransac, source_down, target_down, source_fpfh, target_fpfh = Combiner.global_registration(combined_pcd, pcds[i], voxel_size)
            print("Global registration completed")
            # do ICP
            result_icp = Combiner.refine_registration(combined_pcd, pcds[i], source_fpfh, target_fpfh, voxel_size, result_ransac)
            print("ICP completed")
            # combine the point clouds
            combined_pcd = combined_pcd + pcds[i].transform(result_icp.transformation)
            print("Point clouds combined")
        return combined_pcd
            

    @staticmethod
    def preprocess_point_cloud(pcd, voxel_size):
        print(":: Downsample with a voxel size %.3f." % voxel_size)
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh
    
    @staticmethod
    def prepare_dataset(voxel_size, source, target):
        print(":: Load two point clouds and disturb initial pose.")
        trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        print(":: Apply a rigid transformation to the point cloud")
        source.transform(trans_init)
        
        source_down, source_fpfh = Combiner.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = Combiner.preprocess_point_cloud(target, voxel_size)
        return source_down, target_down, source_fpfh, target_fpfh

    @staticmethod
    def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        print(":: RANSAC registration on downsampled point clouds.")
        print("   Since the downsampling voxel size is %.3f," % voxel_size)
        print("   we use a liberal distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        return result
    
    @staticmethod
    def global_registration(source, target, voxel_size):
        source_down, target_down, source_fpfh, target_fpfh = Combiner.prepare_dataset(voxel_size, source, target)
        result_ransac = Combiner.execute_global_registration(source_down, target_down,
                                                        source_fpfh, target_fpfh,
                                                        voxel_size)
        return result_ransac, source_down, target_down, source_fpfh, target_fpfh
    
    @staticmethod
    def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
        distance_threshold = voxel_size * 0.4
        print(":: Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_icp(
            source, target, distance_threshold, result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        return result
