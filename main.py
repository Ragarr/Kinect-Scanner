from packages.kinect_interface.sensor import Sensor
from packages.kinect_interface.combiner import Combiner
from packages.kinect_interface.frame_processor import FrameProcessor
from packages.utils import *
import open3d as o3d
import time
cf = load_config()

print("Config loaded: ")
print(cf)

sensor = Sensor()
depth_frames = []
print("Sensor created, getting depth frame")
for i in range(3):
    input("Press enter to get next frame")
    
    depth_frames.append(sensor.get_depth_frame())
    
print("Got depth frame, converting to point cloud")

point_clouds = []
for frame in depth_frames:
    point_clouds.append(FrameProcessor.depth_image_to_pointcloud(frame, filter=cf['frame_processing']['filter'], estimate_normals=True, downsample_size=0.01))



print("Point clouds created, combining them")
combined_cloud = Combiner.combine_clouds(point_clouds)
print("Point clouds combined, visualizing")

o3d.visualization.draw_geometries([combined_cloud])