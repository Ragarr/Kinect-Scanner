from packages.kinect_interface.sensor import Sensor
from packages.kinect_interface.assembler import Assembler
from packages.utils import *

cf = load_config()

print("Config loaded")

sensor = Sensor()
print("Sensor created, getting depth frame")
depth_frame = sensor.get_depth_frame()
print("Got depth frame, converting to point cloud")

pcd = Assembler.combine_frames_in_point_cloud([depth_frame], estimate_normals=True, downsample_size=0.01, verbose=False)


import open3d as o3d
# o3d.io.write_point_cloud("E:/Download/open3d-app-windows-amd64-0.18.0/pointcloud.ply", pcd)


o3d.visualization.draw_geometries([pcd])