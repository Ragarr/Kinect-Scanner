import sys 
import os

# add this path to sys.path to import pykinect
sys.path.append(os.path.dirname(__file__))
from pykinect import nui
import numpy as np
import threading
from scipy.ndimage import map_coordinates
import cv2
import numpy as np
from scipy.ndimage import affine_transform
from scipy.optimize import minimize
from sklearn.neighbors import KNeighborsRegressor
np.seterr(divide='ignore', invalid='ignore')

class Sensor:
    def __init__(self, resolution: str="640x480") -> None:
        if resolution == "640x480":
            self.resolution = nui.ImageResolution.resolution_640x480
            self.width = 640
            self.height = 480
        elif resolution == "320x240":
            self.resolution = nui.ImageResolution.resolution_320x240
            self.width = 320
            self.height = 240
        else:
            raise ValueError("Invalid resolution, resolution must be either 640x480 or 320x240")

        self.depth_frame_ready = threading.Event()
        self.color_frame_ready = threading.Event()
        self.capture_depth = threading.Event()  # Flag to control depth capture
        self.capture_color = threading.Event()  # Flag to control color capture
        try:
            self.nui_runtime = nui.Runtime()
        except WindowsError:
            raise WindowsError("Error initializing Kinect runtime. Make sure Kinect is connected.")
        self.camera = nui.Camera(self.nui_runtime)
        self.nui_runtime.depth_frame_ready += self._on_depth_frame_ready
        self.nui_runtime.video_frame_ready += self._on_color_frame_ready
        
        self.nui_runtime.video_stream.open(
            nui.ImageStreamType.Video, 2, self.resolution, nui.ImageType.Color)
        self.nui_runtime.depth_stream.open(
            nui.ImageStreamType.Depth, 2, self.resolution, nui.ImageType.Depth)

        self.depth_frame = np.zeros((self.height, self.width), dtype=np.uint16)
        self.color_frame = np.zeros((self.height, self.width, 4), dtype=np.uint8)

    def __del__(self):
        try:
            self.nui_runtime.close()
            del self.nui_runtime
        except AttributeError:
            pass

    def _on_depth_frame_ready(self, frame):
        if self.capture_depth.is_set():  # Check if depth capture is enabled
            frame.image.copy_bits(self.depth_frame.ctypes.data)
            # change 0 to 65535 in the depth frame
            self.depth_frame_ready.set()

    def _on_color_frame_ready(self, frame):
        if self.capture_color.is_set():  # Check if color capture is enabled
            frame.image.copy_bits(self.color_frame.ctypes.data)
            self.color_frame_ready.set()

    def get_depth_frame(self):
        self.capture_depth.set()  # Enable depth capture
        self.depth_frame_ready.wait(10)
        if not self.depth_frame_ready.is_set():
            self.capture_depth.clear()  # Disable depth capture if timeout
            raise TimeoutError("Depth stream not started or no depth frame ready.")
        self.depth_frame_ready.clear()
        self.capture_depth.clear()  # Disable depth capture after frame is captured
        return self.depth_frame

    def get_color_frame(self):
        self.capture_color.set()  # Enable color capture
        self.color_frame_ready.wait(10)
        if not self.color_frame_ready.is_set():
            self.capture_color.clear()  # Disable color capture if timeout
            raise TimeoutError("Video stream not started or no color frame ready.")
        self.color_frame_ready.clear()
        self.capture_color.clear()  # Disable color capture after frame is captured
        return self.color_frame
    
    def __zoom_image(self, img: np.ndarray , S:float) -> np.ndarray:
        """
        Zoom in or out an image by a factor S using bilinear interpolation.
        Args:
            img (np.ndarray): image to be zoomed
            S (float): zoom factor

        Returns:
            np.ndarray: zoomed image that has same resolution as the input image
        """
        N, M, _ = img.shape
        center_y, center_x = N // 2, M // 2

        # Crear cuadrículas de coordenadas
        y_indices, x_indices = np.indices((N, M))

        # Reescalar estas coordenadas
        y_indices = (y_indices - center_y) / S + center_y
        x_indices = (x_indices - center_x) / S + center_x

        # Inicializar la imagen de salida
        zoomed_img = np.zeros_like(img)

        # Aplicar la interpolación para cada canal RGBA
        for i in range(4):
            zoomed_img[..., i] = map_coordinates(img[..., i], [y_indices, x_indices], order=1, mode='nearest')
        return zoomed_img


    def get_rgbd_frame(self):
        
        zoom_factor = 1.15
        x_rolling_factor = -4
        y_rolling_factor = -20
        
        depth_frame = self.get_depth_frame()
        color_frame = self.get_color_frame()
        
        # color_frame = self.__zoom_image(color_frame, zoom_factor)
                
        transofrmed_rgb_frame = np.zeros((self.height, self.width, 3), dtype=np.uint16)
        transofrmed_depth_frame = np.zeros((self.height, self.width), dtype=np.uint16)
        
        # for each x, y coordinate in the depth frame
        for depth_y in range(self.height):
            for depth_x in range(self.width):
                color_x, color_y= self.camera.get_color_pixel_coordinates_from_depth_pixel(self.resolution, 
                                                                         None, 
                                                                         depth_x, depth_y, 
                                                                         depth_frame[depth_y, depth_x])
                
                # scale the color_x and color_y to the color frame resolution 1280x1024 -> 640x480
                x_scale = 640 / 1280
                y_scale = 480 / 1024
                color_x = int(color_x * x_scale)
                color_y = int(color_y * y_scale)
                               
                if color_x >= 0 and color_x < self.width and color_y >= 0 and color_y < self.height:
                    transofrmed_depth_frame[depth_y, depth_x] = depth_frame[depth_y, depth_x] 
                    transofrmed_rgb_frame[depth_y, depth_x] = color_frame[color_y, color_x][:3]

        # depth_frame, _, _ = self.align_rgb_d(transofrmed_depth_frame, transofrmed_rgb_frame)
        return np.dstack((transofrmed_rgb_frame, depth_frame))
    
    
    def complete_depth_frame(self, depth_frame, k=5):
        """Uses knn to complete the depth frame. The points with depth value 0 are replaced by the average of the k nearest neighbors.

        Args:
            depth_frame (np.ndarray): Depth frame
            k (int, optional): Number of nearest neighbors to consider. Defaults to 5.
        """
        zero_indices = np.argwhere(depth_frame == 0)
        non_zero_indices = np.argwhere(depth_frame != 0)
        non_zero_coords = non_zero_indices[:, [1, 0]]  # Swap x, y for coordinate order
        non_zero_values = depth_frame[depth_frame != 0]
        
        knn = KNeighborsRegressor(n_neighbors=k)
        knn.fit(non_zero_coords, non_zero_values)
        
        zero_coords = zero_indices[:, [1, 0]]  # Swap x, y for coordinate order
        zero_values = knn.predict(zero_coords)
        
        depth_frame[zero_indices[:, 0], zero_indices[:, 1]] = zero_values
        return depth_frame
    


        
    
    def align_rgb_d(self, depth_image, rgb_image ):
        """Aligns the depth image with the rgb image using ORB feature matching and homography transformation.

        Args:
            img_depth (np.ndarray): Depth image
            img_rgb (np.ndarray): RGB image
            
        Returns:
            np.ndarray: Aligned depth image
            np.ndarray: RGB image
            np.ndarray: Transformation matrix
        """    
        # Función para calcular la Entropía de una imagen
        def image_entropy(image):
            hist = np.histogram(image, bins=256, range=(0, 255))[0]
            hist = hist / np.sum(hist)  # Normalizar el histograma
            return -np.sum(hist * np.log(hist + 1e-15))  # Evitar log(0)

        # Función para calcular la Información Mutua entre dos imágenes
        def mutual_information(image1, image2):
            hist_2d, _, _ = np.histogram2d(image1.flatten(), image2.flatten(), bins=256, range=[[0, 255], [0, 255]])
            return image_entropy(image1) + image_entropy(image2) - image_entropy(hist_2d)

        # Función para optimizar la transformación usando Información Mutua
        def optimize_transform(image1, image2, initial_params):
            def loss(params):
                transformed = affine_transform(image2, params.reshape((2, 3)))
                return -mutual_information(image1, transformed)

            result = minimize(loss, initial_params, method='Nelder-Mead')
            return result.x


        def initial_transform():
            return np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0])  # Sin transformación inicial

        
        # relleando los valores de la imagen de profundidad
        depth_image=self.complete_depth_frame(depth_image)

        img_color_gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        print("Optimizing depth transform...")
        params = optimize_transform(img_color_gray, depth_image, initial_transform())
        print("Transforming depth image...")
        aligned_depth = affine_transform(depth_image, params.reshape((2, 3)))
        print("Done")
        return aligned_depth, rgb_image, params
        
        
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    sensor = Sensor()
    rgbd_frame = sensor.get_rgbd_frame()
    # show color and depth frames
    plt.figure()
    plt.subplot(121)
    plt.imshow(rgbd_frame[..., :3])
    plt.title("Color Frame")
    plt.subplot(122)
    plt.imshow(rgbd_frame[..., 3], cmap='gray')
    plt.title("Depth Frame")
    plt.show()