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
            self.height = 480
            self.width = 640
        elif resolution == "320x240":
            self.height = 240
            self.width = 320
        else:
            raise ValueError("Invalid resolution, resolution must be either 640x480 or 320x240")

        self.depth_frame_ready = threading.Event()
        self.color_frame_ready = threading.Event()
        self.capture_depth = threading.Event()  # Flag to control depth capture
        self.capture_color = threading.Event()  # Flag to control color capture
        
        self.kinect = nui.Runtime()
        self.kinect.depth_frame_ready += self._on_depth_frame_ready
        self.kinect.video_frame_ready += self._on_color_frame_ready

        self.kinect.video_stream.open(
            nui.ImageStreamType.Video, 2, nui.ImageResolution.resolution_640x480 if resolution == "640x480" else nui.ImageResolution.resolution_320x240, nui.ImageType.Color)
        self.kinect.depth_stream.open(
            nui.ImageStreamType.Depth, 2, nui.ImageResolution.resolution_640x480 if resolution == "640x480" else nui.ImageResolution.resolution_320x240, nui.ImageType.Depth)

        self.depth_frame = np.zeros((self.height, self.width), dtype=np.uint16)
        self.color_frame = np.zeros((self.height, self.width, 4), dtype=np.uint8)

    def __del__(self):
        try:
            self.kinect.close()
            del self.kinect
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

        rgbad_frame = np.zeros((self.height, self.width, 4), dtype=np.uint16)
        rgbad_frame[:, :, :3] = np.array(color_frame, dtype=np.uint16)[:, :, :3]
        # remove the alpha channel
        # convert BGR to RGB
        rgbad_frame[:, :, :3] = rgbad_frame[:, :, :3][:, :, ::-1]
        
        rgbad_frame[:, :, 3] = depth_frame
        return rgbad_frame    

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    sensor = Sensor()
    rgbad_frame = sensor.get_rgbd_frame()
    # show color and depth frames
    plt.figure()
    plt.imshow(rgbad_frame[:, :, :3])
    plt.title("Color Frame")
    plt.axis("off")
    plt.figure()
    plt.imshow(rgbad_frame[:, :, 3], cmap="gray")
    plt.title("Depth Frame")
    plt.axis("off")
    plt.show()
    
    
    
