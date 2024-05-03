import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import threading

class Worker(Node):
    def __init__(self):
        super().__init__("rs_data_fetcher")

        self.color_image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.color_image_callback, 1
        )
        self.depth_image_sub = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.depth_image_callback, 1
        )
        self.color_image = None
        self.depth_image = None

    def color_image_callback(self, msg):
        self.color_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )

    def depth_image_callback(self, msg):
        self.depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width
        ).astype(np.float32) / 1000.0  # realsense default unit is mm

class RSDataFetcher:
    def __init__(self):
        rclpy.init()
        self.rs_data_fetcher = Worker()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.rs_data_fetcher)

        # create a thread to run the node
        self.rs_data_fetcher_thread = threading.Thread(target=self.executor.spin)
        self.rs_data_fetcher_thread.start()

    def get_data(self):
        return self.rs_data_fetcher.color_image, self.rs_data_fetcher.depth_image
    
    def destroy(self):
        self.executor.shutdown()
        self.rs_data_fetcher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    import cv2
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
