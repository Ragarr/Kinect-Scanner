import sys 
import os

# add this path to sys.path to import pykinect
sys.path.append(os.path.dirname(__file__))
from pykinect import nui
import numpy as np
import threading

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
        
        self.nui_runtime = nui.Runtime()
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

    def get_rgbd_frame(self):
        depth_frame = self.get_depth_frame()
        color_frame = self.get_color_frame()
        rgbd_frame = np.zeros((self.height, self.width, 4), dtype=np.uint16)
        rgbd_frame[:, :, :3] = np.array(color_frame, dtype=np.uint16)[:, :, :3]
        
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
                    rgbd_frame[color_y, color_x, 3] = depth_frame[depth_y, depth_x]
                
        # change BGR to RGB
        rgbd_frame[:, :, 2] = color_frame[:, :, 0]
        rgbd_frame[:, :, 1] = color_frame[:, :, 1]
        rgbd_frame[:, :, 0] = color_frame[:, :, 2]
        
                            
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
    
    
    
