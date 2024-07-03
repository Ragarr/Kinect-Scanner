import numpy as np
import open3d as o3d


class FrameProcessor:

    # Load the pol2car parameters from config/pol2car.npz
    pol2car_params = np.load("config/pol2car2.npz")

    @staticmethod
    def depth_image_to_pointcloud(
        frame: np.ndarray,
        filter=[None, None, None],
        estimate_normals=True,
        downsample_size=0,
    ) -> o3d.geometry.PointCloud:
        """
        Converts a depth frame (optionally with RGB) to a point cloud.

        Args:
            frame (np.ndarray): The depth frame to convert. It can be either a 2D array (depth only) or a 4D array (r, g, b, depth).
            filter (list, optional): A list of filters to apply to the depth frame. Defaults to [None, None, None].
            estimate_normals (bool, optional): Whether to estimate normals for the point cloud. Defaults to True.
            downsample_size (float, optional): Voxel size for downsampling the point cloud. Defaults to 0.

        Returns:
            o3d.geometry.PointCloud: The converted point cloud.

        Raises:
            ValueError: If the frame is not a valid shape (2D or 4D).
        """
        if frame.ndim == 2:
            # If the frame is 2D (depth only)
            depth_frame = frame
            color_frame = None
        elif frame.ndim == 3 and frame.shape[2] == 4:
            # If the frame is 4D (r, g, b, depth)
            color_frame = frame[..., :3]
            depth_frame = frame[..., 3]
        else:
            raise ValueError(
                "Frame must be either 2D (depth only) or 4D (r, g, b, depth)."
            )

        preprocessed_frame = FrameProcessor.preprocess_img(depth_frame)
        coords = FrameProcessor.depth_image_to_scientific_coordinates(
            preprocessed_frame, filter=filter
        )

        if coords.shape[0] == 0:
            print("No points in the point cloud")

        coords = np.ascontiguousarray(coords, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)

        if color_frame is not None:
            colors = FrameProcessor.depth_image_to_colors(color_frame, filter=filter)
            pcd.colors = o3d.utility.Vector3dVector(colors)

        if estimate_normals:
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )

        if downsample_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=downsample_size)

        # remove the points that are pure black
        pcd = pcd.select_by_index(
            np.where(np.sum(np.asarray(pcd.colors), axis=1) > 0)[0]
        )

        return pcd

    @staticmethod
    def preprocess_img(original_img: np.ndarray) -> np.ndarray:
        """
        Preprocesses the original image by filtering the depth image.

        Args:
            original_img (np.ndarray): The original image to be preprocessed.

        Returns:
            np.ndarray: The preprocessed image.
        """
        img = FrameProcessor.filter_depth_image(original_img)

        return img

    @staticmethod
    def raw_depth_to_radial_depth(img):
        """
        Converts raw depth values to radial depth values.

        Args:
            img (numpy.ndarray): The input image containing raw depth values.

        Returns:
            numpy.ndarray: The image with converted radial depth values.
        """
        r_scale, r_offset = FrameProcessor.pol2car_params["raw_z_to_r"]
        return img * r_scale + r_offset


    @staticmethod
    def depth_image_to_scientific_coordinates(
        image: np.ndarray, filter=[None, None, None], flatten=True
    ) -> np.ndarray:
        """
        Converts a depth image to scientific coordinates.

        Args:
            image (np.ndarray): The depth image to convert.
            filter (list, optional): A list of filter ranges for each coordinate (x, y, z). Defaults to [None, None, None].
            flatten (bool, optional): Whether to flatten the resulting points array. Defaults to True.

        Returns:
            np.ndarray: The converted points in scientific coordinates.
        """
        r = FrameProcessor.raw_depth_to_radial_depth(image)
        x = r * FrameProcessor.pol2car_params["r_to_x"]
        y = r * FrameProcessor.pol2car_params["r_to_y"]
        z = r * FrameProcessor.pol2car_params["r_to_z"]
        points = np.dstack((x, y, z)).reshape(-1, 3)
        points = FrameProcessor.scientific_coordinates_to_standard(points)

        for i in range(3):
            if filter[i] is not None:
                points = points[
                    (points[:, i] > filter[i][0]) & (points[:, i] < filter[i][1]), :
                ]

        if flatten:
            return points
        else:
            return points[0, :], points[1, :], points[2, :]

    @staticmethod
    def depth_image_to_colors(
        color_image: np.ndarray, filter=[None, None, None]
    ) -> np.ndarray:
        """
        Converts a depth image to colors.

        Args:
            color_image (np.ndarray): The input color image.
            filter (list, optional): The filter to apply to the color image. Defaults to [None, None, None].

        Returns:
            np.ndarray: The converted color image.
        """
        color_image = color_image / 255.0  # Normalize RGB values to [0, 1]
        return color_image.reshape(-1, 3)

    @staticmethod
    def print_pointcloud_mat_stats(mat):
        """
        Prints the minimum and maximum values of the x, y, and z coordinates in the given point cloud matrix.

        Args:
            mat (numpy.ndarray): The point cloud matrix.

        Returns:
            None
        """
        print(
            "min x: {0:.02f}, max x: {3:.02f}, min y: {1:.02f}, max y: {4:.02f}, min z: {2:.02f}, max z: {5:.02f}".format(
                *mat.min(axis=0), *mat.max(axis=0)
            )
        )

    @staticmethod
    def scientific_coordinates_to_standard(mat: np.ndarray) -> np.ndarray:
        """
        Converts scientific coordinates to standard coordinates.

        Args:
            mat (np.ndarray): The input matrix representing scientific coordinates.

        Returns:
            np.ndarray: The transformed matrix representing standard coordinates.
        """
        transformed_mat = (mat - np.array([[0, 2, 0]])) * np.array([[1, -1, 1]])
        transformed_mat = transformed_mat[:, [0, 2, 1]]
        transformed_mat = (
            transformed_mat
            @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
            @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        )
        return transformed_mat

    @staticmethod
    def filter_depth_image(img, min_distance=0, max_distance=255, flip_z=False):
        """
        Applies a filter to the depth image.

        Args:
            img (numpy.ndarray): The depth image to be filtered.
            min_distance (int, optional): The minimum distance value for the filter. Defaults to 0.
            max_distance (int, optional): The maximum distance value for the filter. Defaults to 255.
            flip_z (bool, optional): Whether to flip the z-axis of the image. Defaults to False.

        Returns:
            numpy.ndarray: The filtered depth image.
        """
        img = (img - np.min(img)) / (np.max(img) - np.min(img)) * (
            max_distance - min_distance
        ) + min_distance
        if flip_z:
            img = max_distance - img
        return img

    @staticmethod
    def reduce_noise(pcd: o3d.geometry.PointCloud, noise_eps=0.02, min_neighbors=50):
        """
        Reduces noise in a point cloud by removing outliers based on the specified parameters.

        Args:
            pcd (o3d.geometry.PointCloud): The input point cloud.
            noise_eps (float, optional): The radius for outlier removal. Points with fewer neighbors within this radius will be considered outliers. Defaults to 0.02.
            min_neighbors (int, optional): The minimum number of neighbors required for a point to be considered an inlier. Defaults to 50.

        Returns:
            o3d.geometry.PointCloud: The cleaned point cloud with reduced noise.
        """
        clean_pcd = pcd.remove_radius_outlier(nb_points=min_neighbors, radius=noise_eps)
        return clean_pcd


if __name__ == "__main__":
    print("This file is not meant to be run directly. Import it into another file.")
