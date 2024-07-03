import sys 
import os

# add this path to sys.path to import pykinect
sys.path.append(os.path.dirname(__file__))
from pykinect import nui
import numpy as np
import threading
from scipy.ndimage import map_coordinates


class Sensor:
    class Sensor:
        def __init__(self, resolution: str="640x480") -> None:
            """
            Initializes a Sensor object.

            Args:
                resolution (str, optional): The resolution of the sensor. Defaults to "640x480".

            Raises:
                ValueError: If an invalid resolution is provided.
                WindowsError: If the Kinect runtime cannot be initialized.
            """
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
            """
            Destructor method for the Sensor class.
            Closes the NUI runtime and deletes the NUI runtime object.
            """
            try:
                self.nui_runtime.close()
                del self.nui_runtime
            except AttributeError:
                pass

    def _on_depth_frame_ready(self, frame):
        """
        Callback function for handling depth frame updates.

        Args:
            frame: The depth frame object containing the updated depth data.

        Returns:
            None
        """
        if self.capture_depth.is_set():  # Check if depth capture is enabled
            frame.image.copy_bits(self.depth_frame.ctypes.data)
            # change 0 to 65535 in the depth frame
            self.depth_frame_ready.set()

    def _on_color_frame_ready(self, frame):
        """
        Callback method triggered when a color frame is ready.

        Args:
            frame: The color frame object.

        Returns:
            None
        """
        if self.capture_color.is_set():  # Check if color capture is enabled
            frame.image.copy_bits(self.color_frame.ctypes.data)
            self.color_frame_ready.set()

    def get_depth_frame(self):
        """
        Retrieves a depth frame from the Kinect sensor.

        Returns:
            The depth frame captured by the sensor.

        Raises:
            TimeoutError: If the depth stream is not started or no depth frame is ready within the timeout period.
        """
        self.capture_depth.set()  # Enable depth capture
        self.depth_frame_ready.wait(10)
        if not self.depth_frame_ready.is_set():
            self.capture_depth.clear()  # Disable depth capture if timeout
            raise TimeoutError("Depth stream not started or no depth frame ready.")
        self.depth_frame_ready.clear()
        self.capture_depth.clear()  # Disable depth capture after frame is captured
        return self.depth_frame

    def get_color_frame(self):
        """
        Retrieves a color frame from the Kinect sensor.

        Returns:
            The color frame captured by the Kinect sensor.

        Raises:
            TimeoutError: If the video stream is not started or no color frame is ready within the timeout period.
        """
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
        """
        Retrieves the RGBD frame from the Kinect sensor.

        Returns:
            np.ndarray: The RGBD frame, represented as a numpy array of shape (height, width, 4).
                        The fourth channel represents the depth values.
        """
        zoom_factor = 1.15
        x_rolling_factor = -4
        y_rolling_factor = -20
        
        depth_frame = self.get_depth_frame()
        color_frame = self.get_color_frame()
        
        color_frame = self.__zoom_image(color_frame, zoom_factor)
        
        rgbd_frame = np.zeros((self.height, self.width, 4), dtype=np.uint16)
        
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
                    rgbd_frame[depth_y, depth_x, 3] = depth_frame[depth_y, depth_x] 
                    rgbd_frame[depth_y, depth_x, 2] = color_frame[color_y, color_x, 0]  
                    rgbd_frame[depth_y, depth_x, 1] = color_frame[color_y, color_x, 1]
                    rgbd_frame[depth_y, depth_x, 0] = color_frame[color_y, color_x, 2]
                
        # change BGR to RGB
        rgbd_frame[:, :, 2] = color_frame[:, :, 0]
        rgbd_frame[:, :, 1] = color_frame[:, :, 1]
        rgbd_frame[:, :, 0] = color_frame[:, :, 2]
        
        # move just the color 10 px to the left, without reintroducting the same pixels
        rgbd_frame[:, :, :3] = np.roll(rgbd_frame[:, :, :3], x_rolling_factor, axis=1)
        rgbd_frame[:, :, :3] = np.roll(rgbd_frame[:, :, :3], y_rolling_factor, axis=0)
        
        # remove the 0 depth values and the reintruduced pixels
        rgbd_frame[rgbd_frame[:, :, 3] == 0] = 0
        rgbd_frame[rgbd_frame[:, :, 3] == 65535] = 0
        
        return rgbd_frame

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    sensor = Sensor()
    rgbd_frame = sensor.get_rgbd_frame()
    # show color and depth frames
    plt.figure()
    plt.imshow(rgbd_frame[:, :, :3])
    plt.title("Color Frame")
    plt.axis("off")
    plt.figure()
    plt.imshow(rgbd_frame[:, :, 3], cmap="gray")
    plt.title("Depth Frame")
    plt.axis("off")
    plt.show()
    
    
    
