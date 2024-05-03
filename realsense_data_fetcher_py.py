## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

class RSDataFetcher:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

    def get_data(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data(), dtype=np.float32) / 1000.0  # realsense default unit is mm
        print(np.max(depth_image), np.min(depth_image))
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image

    def destroy(self):
        # Stop streaming
        self.pipeline.stop()

if __name__ == "__main__":
    rs_data_fetcher = RSDataFetcher()
    while True:
        color_image, depth_image = rs_data_fetcher.get_data()
        if color_image is None or depth_image is None:
            continue
        cv2.imshow("color_image", color_image)
        cv2.imshow("depth_image", depth_image)
        if cv2.waitKey(0) & 0xFF == ord("q"):
            break
    rs_data_fetcher.destroy()