import sys 
import os

# add this path to sys.path to import pykinect
sys.path.append(os.path.dirname(__file__))
from pykinect import nui
import numpy as np
import threading
import functools 

_depth_semaphore = threading.Semaphore(0)
_color_semaphore = threading.Semaphore(0)

_depth_complete = threading.Event()
_color_complete = threading.Event()
_video_stream_started = threading.Event()
_depth_stream_started = threading.Event()


class Sensor:
    def __init__(self, resolution: str="640x480") -> None:
        if resolution == "640x480":
            self.height = 480
            self.width = 640
            self.kinect = nui.Runtime()
            self.kinect.depth_frame_ready += functools.partial(self._on_depth_frame_ready)
            self.kinect.depth_stream.open(nui.ImageStreamType.Depth, 2, nui.ImageResolution.resolution_640x480, nui.ImageType.Depth)
            self.kinect.video_frame_ready += functools.partial(self._on_color_frame_ready)
            self.kinect.video_stream.open(nui.ImageStreamType.Video, 2, nui.ImageResolution.resolution_640x480, nui.ImageType.Color)
            self.depth_frame = np.zeros((480, 640), dtype=np.uint16)
            self.color_frame = np.zeros((480, 640, 4), dtype=np.uint8)
        elif resolution == "320x240":
            self.height = 240
            self.width = 320
            self.kinect = nui.Runtime()
            self.kinect.depth_frame_ready += functools.partial(self._on_depth_frame_ready)
            self.kinect.depth_stream.open(nui.ImageStreamType.Depth, 2, nui.ImageResolution.resolution_320x240, nui.ImageType.Depth)
            self.kinect.video_frame_ready += functools.partial(self._on_color_frame_ready)
            self.kinect.video_stream.open(nui.ImageStreamType.Video, 2, nui.ImageResolution.resolution_320x240, nui.ImageType.Color)
            self.depth_frame = np.zeros((240, 320), dtype=np.uint16)
            self.color_frame = np.zeros((240, 320, 4), dtype=np.uint8)
        else:
            raise ValueError("Invalid resolution, resolution must be either 640x480 or 320x240")

    def __del__(self):
        self.kinect.close()
        del self.kinect       
    
    def _on_depth_frame_ready(self, frame):
        global _depth_semaphore, _depth_complete, _depth_stream_started
        _depth_stream_started.set()
        print("depth frame ready")
        _depth_semaphore.acquire()
        frame.image.copy_bits(self.depth_frame.ctypes.data)
        _depth_complete.set()
        
    
    def get_depth_frame(self):
        global _depth_semaphore, _depth_complete, _depth_stream_started
        _depth_stream_started.wait()
        _depth_complete.clear()
        _depth_semaphore.release()
        print("Waiting for depth flag")
        _depth_complete.wait()
        print("Depth flag set")
        return self.depth_frame
    
    def _on_color_frame_ready(self, frame):
        global _color_semaphore, _color_complete, _video_stream_started
        _video_stream_started.set()
        print("color frame ready")
        _color_semaphore.acquire()
        frame.image.copy_bits(self.color_frame.ctypes.data)
        _color_complete.set()
    
    def get_color_frame(self):
        global _color_semaphore, _color_complete, _video_stream_started
        _video_stream_started.wait()
        _color_complete.clear()
        _color_semaphore.release()
        print("Waiting for color flag")
        _color_complete.wait() 
        print("Color flag set")       
        return self.color_frame
    
    def get_rgbd_frame(self):
        depth_frame = self.get_depth_frame()
        color_frame = self.get_color_frame()
        
        rgbad_frame = np.zeros((self.height, self.width, 5), dtype=np.uint16)
        # transfer color frame to the first 4 channels, convert to uint16
        rgbad_frame[:, :, :4] = np.array(color_frame, dtype=np.uint16)
        # transfer depth frame to the last channel, convert to uint16
        rgbad_frame[:, :, 4] = depth_frame
        
        return rgbad_frame    
    

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    sensor = Sensor()
    rgbad_frame = sensor.get_rgbd_frame()
    plt.imshow(np.array(rgbad_frame[:, :, :3]), interpolation='nearest')
    plt.show()
    