from packages.utils import *

from packages.kinect_interface.depth_sensor import DepthSensor
import matplotlib.pyplot as plt


cf = load_config()

print(cf.get("sensors", "num_sensors"))

sensor = DepthSensor()
depth_frame = sensor.get_depth_frame()
plt.imshow(depth_frame)
plt.show()


