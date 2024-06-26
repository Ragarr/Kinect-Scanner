from pykinect import nui
import numpy as np
import threading
import functools 
import ctypes as c


_depth_semaphore = threading.Semaphore(0)
_processing_complete = threading.Event()

class DepthSensor:
    def __init__(self, resolution: str="640x480") -> None:
        if resolution == "640x480":
            self.height = 480
            self.width = 640
            self.sensor = nui.Runtime()
            self.sensor.depth_frame_ready += functools.partial(self._on_depth_frame_ready)
            self.sensor.depth_stream.open(nui.ImageStreamType.Depth, 2, nui.ImageResolution.resolution_640x480, nui.ImageType.Depth)
            self.depth_frame = np.zeros((480, 640), dtype=np.uint16)
        elif resolution == "320x240":
            self.height = 240
            self.width = 320
            self.sensor = nui.Runtime()
            self.sensor.depth_frame_ready += functools.partial(self._on_depth_frame_ready)
            self.sensor.depth_stream.open(nui.ImageStreamType.Depth, 2, nui.ImageResolution.resolution_320x240, nui.ImageType.Depth)
            self.depth_frame = np.zeros((240, 320), dtype=np.uint16)
        else:
            raise ValueError("Invalid resolution, resolution must be either 640x480 or 320x240")
    
    def __del__(self):
        self.sensor.close()
        del self.sensor       
    
    def _on_depth_frame_ready(self, frame):
        global _depth_semaphore, _processing_complete
        _depth_semaphore.acquire()
        # frame.image is:
        '''
        class PlanarImage(ctypes.c_voidp):
            """Represents a video image."""
            _BufferLen = ctypes.WINFUNCTYPE(ctypes.HRESULT, ctypes.c_int32)(3, 'BufferLen')
            _Pitch = ctypes.WINFUNCTYPE(ctypes.HRESULT, ctypes.c_int32)(4, 'Pitch')
            _LockRect = ctypes.WINFUNCTYPE(ctypes.HRESULT, ctypes.c_uint, ctypes.POINTER(_NuiLockedRect), ctypes.c_voidp, ctypes.c_uint32)(5, '_LockRect')
            _GetLevelDesc = ctypes.WINFUNCTYPE(ctypes.HRESULT, ctypes.c_uint32, ctypes.POINTER(_NuiSurfaceDesc))(6, '_GetLevelDesc')
            _UnlockRect = ctypes.WINFUNCTYPE(ctypes.HRESULT, ctypes.c_uint32)(7, '_UnlockRect')
        
                @property
                def bits(self):
                    buffer = (ctypes.c_byte * self.buffer_length)()
                    self.copy_bits(buffer)
                    return buffer
        '''
        frame.image.copy_bits(self.depth_frame.ctypes.data)
        _processing_complete.set()
            
    
    def get_depth_frame(self):
        global _depth_semaphore, _processing_complete
        _processing_complete.clear()
        _depth_semaphore.release()
        _processing_complete.wait()
        return self.depth_frame
    

    

if __name__ == "__main__":
    sensor = DepthSensor()
    depth_frame = sensor.get_depth_frame()
    import matplotlib.pyplot as plt
    plt.imshow(depth_frame)
    plt.show()