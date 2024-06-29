from packages.kinect_interface.sensor import Sensor
from packages.kinect_interface.frame_processor import FrameProcessor
from packages.utils import *
import open3d as o3d
import time
cf = load_config()

print("Config loaded: ")
print(cf)

sensor = Sensor()

frame = sensor.get_rgbd_frame()

ptc = FrameProcessor.depth_image_to_pointcloud(frame)

o3d.visualization.draw_geometries([ptc])