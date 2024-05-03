#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraImagePublisher(Node):
    def __init__(self):
        super().__init__("Image_publisher")
        self.publisher = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        flipped_frame = cv2.flip(frame, 1)

        if ret:
            compressed_frame = self.cv_bridge.cv2_to_compressed_imgmsg(flipped_frame, 'jpeg')
            self.publisher.publish(compressed_frame)
            self.get_logger().info("Publishing video frame")

def main():
    rclpy.init()
    node = CameraImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
     main()
