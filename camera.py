import pyrealsense2 as rs
import numpy as np
import cv2, os
from PIL import Image

class RealSenseCapture:
    def __init__(self, width=640, height=480, fps=30, enable_align=True, save_path=""):
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_align = enable_align
        self.save_path = save_path
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path, exist_ok=True)
 
    def get_rgbd(self, idx=0):
        """
        获取单帧 RGB (H,W,3) uint8 和 Depth (H,W) uint16
        完成后自动关闭 pipeline，极低 CPU 开销
        """
        pipeline = rs.pipeline()
        config = rs.config()
 
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
 
        profile = pipeline.start(config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        color_sensor.set_option(rs.option.white_balance, 5500)

        align = rs.align(rs.stream.color) if self.enable_align else None
 
        for _ in range(5):
            frames = pipeline.wait_for_frames()
 
        frames = pipeline.wait_for_frames()
 
        if self.enable_align:
            frames = align.process(frames)
 
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
 
        rgb = np.asanyarray(color_frame.get_data())[..., ::-1]
        depth = np.asanyarray(depth_frame.get_data())
 
        pipeline.stop()
        Image.fromarray(rgb).save(os.path.join(self.save_path, "rgb.png"))
        Image.fromarray(depth).save(os.path.join(self.save_path, "depth.png"))
        return rgb, depth

if __name__ == '__main__':
    cam = RealSenseCapture(width=640, height=480, fps=30, save_path="./log")
    rgb, depth = cam.get_rgbd()
    print(rgb.shape)   # (480, 640, 3)
    print(depth.shape) # (480, 640)