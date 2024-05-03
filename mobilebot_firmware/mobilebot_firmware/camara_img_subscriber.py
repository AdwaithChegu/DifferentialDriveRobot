#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraImageSubscriber(Node):
    def __init__(self):
        super().__init__("camera_image_subscriber")
        self.sub = self.create_subscription(CompressedImage, 'image_raw/compressed', self.listenerCallback, 10)
        self.cv_bridge = CvBridge()

    def object_detect(self, image):
        cv2.imshow("object", image)
        cv2.waitKey(10)

    def listenerCallback(self, data):
        self.get_logger().info("Receiving compressed video frame")
        image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        self.object_detect(image)

def main(args=None):
    rclpy.init(args=args)
    node = CameraImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
