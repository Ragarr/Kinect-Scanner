from packages.kinect_interface.sensor import Sensor
from packages.kinect_interface.frame_processor import FrameProcessor
from packages.utils import *
import open3d as o3d
import numpy as np
cf = load_config()

print("Config loaded: ")
print(cf)

sensor = Sensor()

frame = sensor.get_rgbd_frame()



ptc = FrameProcessor.depth_image_to_pointcloud(frame, cf['pcd']['filter'], cf['pcd']['estimate_normals'], cf['pcd']['downsample_size'])


FrameProcessor.reduce_noise(ptc, cf['pcd']['noise_eps'], cf['pcd']['min_neighbors'])

o3d.visualization.draw_geometries([ptc])