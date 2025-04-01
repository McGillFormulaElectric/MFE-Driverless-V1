#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# TODO: change this to yours
PATH_TO_REPO = "/home/five/Documents/MFE/MFE25"

class FileLoaderNode(Node):
    def __init__(self):
        super().__init__('file_loader_node')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  
            depth=10,                                  # keep last 10 messages
            reliability=ReliabilityPolicy.RELIABLE,     # reliable
        )

        self.image_publishing = self.create_publisher(Image, "camera/image/raw", qos_profile)
        self.cap = cv2.VideoCapture(PATH_TO_REPO + "/MFE-Driverless-V1/ros2/src/mfe_perception/vision_cone_detector/resource/video1.mp4")
        self.bridge = CvBridge()

        if self.cap.get(cv2.CAP_PROP_FPS) != 0 : fps = self.cap.get(cv2.CAP_PROP_FPS)
        else : fps = 0.03

        self.timer = self.create_timer(1 / fps, self.publish_frame)

    def publish_frame(self):
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file.")
            return
        
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publishing.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = FileLoaderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

