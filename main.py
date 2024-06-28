from packages.kinect_interface.sensor import Sensor
from packages.kinect_interface.combiner import Combiner
from packages.kinect_interface.frame_processor import FrameProcessor
from packages.utils import *
import open3d as o3d
import time
import numpy as np
cf = load_config()

print("Config loaded: ")
print(cf)

sensor = Sensor()

for i in range(10):
    print("Capturing frame ", i)
    frame = sensor.get_depth_frame()
    # save frame to file
    np.save("frames/frame" + str(i), frame)
    