from packages.kinect_interface.depth_sensor import DepthSensor
from packages.kinect_interface.frame_processor import FrameProcessor
from packages.utils import *

cf = load_config()

print(cf.get("sensors", "num_sensors"))

sensor = DepthSensor()
depth_frame = sensor.get_depth_frame()

df = FrameProcessor.depth_image_to_pointcloud(depth_frame)
