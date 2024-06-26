import numpy as np
from pyntcloud import PyntCloud
import pandas as pd

class FrameProcessor:
    
    # load the pol2car parameters from config/pol2car.npz
    pol2car_params = np.load('config/pol2car2.npz')
    
    @staticmethod
    def depth_image_to_pointcloud(frame: np.ndarray) -> PyntCloud:
        '''
        Converts a depth frame to a point cloud
        '''
        # Get the height and width of the frame
        height, width = frame.shape
        preprocessed_frame = FrameProcessor.filter_depth_image(frame)
        coords = FrameProcessor.depth_image_to_scientific_coordinates(preprocessed_frame)
        pcd = PyntCloud(pd.DataFrame(coords, columns=['x', 'y', 'z'])) 
        return pcd
        
        
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
    def scientific_coordinates_to_standard(mat: np.ndarray) -> np.ndarray:
        # center y and invert:
        transformed_mat = (mat - np.array([[0, 2, 0]])) * np.array([[1, -1, 1]])
        # swap y and z:
        transformed_mat = transformed_mat[:, [0, 2, 1]]
        return transformed_mat
    
    @staticmethod
    def filter_depth_image(img: np.ndarray, min_depth: float=0, max_depth: float=255, flip_z:bool=False) -> np.ndarray:
        '''
        Filters a depth image based on a minimum and maximum depth
        '''
        # Create a mask for the depth image
        mask = np.logical_and(img > min_depth, img < max_depth)
        # Apply the mask to the depth image
        filtered_img = img * mask
        # Flip the z-axis if required
        if flip_z:
            filtered_img = max_depth - filtered_img
        return filtered_img
    
        
        
        
    

if __name__ == "__main__":
    print("This file is not meant to be run directly. Import it into another file.")