import numpy as np
import open3d as o3d
class FrameProcessor:
    
    # load the pol2car parameters from config/pol2car.npz
    pol2car_params = np.load('config/pol2car2.npz')
    
    @staticmethod
    def depth_image_to_pointcloud(frame: np.ndarray):
        '''
        Converts a depth frame to a point cloud
        '''
        # Get the height and width of the frame

        preprocessed_frame = FrameProcessor.preprocess_img(frame)

        coords = FrameProcessor.depth_image_to_scientific_coordinates(preprocessed_frame)
        coords = np.ascontiguousarray(coords, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
        return pcd
        
    
    @staticmethod
    def preprocess_img(original_img:np.ndarray) -> np.ndarray:
        # Step 2: Filter (crop) the depth:
        img = FrameProcessor.filter_depth_image(original_img)
        return img
    
    @staticmethod
    def raw_depth_to_radial_depth(img):
        r_scale, r_offset = FrameProcessor.pol2car_params['raw_z_to_r']
        return img * r_scale + r_offset    
    
    @staticmethod
    def depth_image_to_scientific_coordinates(image: np.ndarray, filter = [None, None, None], flatten:bool=True) -> np.ndarray:
        """Converts a depth image to scientific coordinates

        Args:
            image (np.ndarray):  A numpy array containing the depth image
            filter (list, optional): a list of 3 tuples, each tuple contains the minimum and maximum values for the x, y, and z coordinates. Defaults to [None, None, None].
            flatten (bool, optional): A flag to determine if the points should be returned as a single array or as separate arrays. Defaults to True.
        Returns:
            np.ndarray:  A numpy array containing the scientific coordinates of the points
        """
        r = FrameProcessor.raw_depth_to_radial_depth(image)
        x = r * FrameProcessor.pol2car_params['r_to_x']
        y = r * FrameProcessor.pol2car_params['r_to_y']
        z = r * FrameProcessor.pol2car_params['r_to_z']
        points = points = np.dstack((x, y, z)).reshape(-1, 3)
        points = FrameProcessor.scientific_coordinates_to_standard(points)

        for i in range(3):
            if filter[i] is not None:
                # filter the points based on the filter
                points = points[(points[:, i] > filter[i][0]) & (points[:, i] < filter[i][1]), :]
        if flatten:
            # return the points as a single array
            return points
        else:
            # return the points as separate arrays
            return points[0, :], points[1, :], points[2, :]
        
    
    @staticmethod
    def print_pointcloud_mat_stats(mat):
        print('min x: {0:.02f}, max x: {3:.02f}, min y: {1:.02f}, max y: {4:.02f}, min z: {2:.02f}, max z: {5:.02f}'.format(*mat.min(axis=0), *mat.max(axis=0)))
    
    @staticmethod
    def scientific_coordinates_to_standard(mat: np.ndarray) -> np.ndarray:
        # center y and invert:
        transformed_mat = (mat - np.array([[0, 2, 0]])) * np.array([[1, -1, 1]])
        # swap y and z:
        transformed_mat = transformed_mat[:, [0, 2, 1]]
        
        # rotate 90 degrees around z axis:
        transformed_mat = transformed_mat @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        # transformed_mat = transformed_mat @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        return transformed_mat
    
    @staticmethod
    def filter_depth_image(img, min_distance=0, max_distance=255, flip_z=False):
        # kinect depth images are 0 for far away:
        # normalize the depth image to be between min_distance and max_distance
        img = (img - np.min(img)) / (np.max(img) - np.min(img)) * (max_distance - min_distance) + min_distance
        if flip_z:
            img = max_distance - img
        return img
    

if __name__ == "__main__":
    print("This file is not meant to be run directly. Import it into another file.")