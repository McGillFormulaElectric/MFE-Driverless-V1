#!/usr/bin/env python3

import threading
import cv2
import torch
import numpy as np
import queue

from typing import Generator

import rclpy
from rclpy.node import Node
from ultralytics import YOLO

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

# installs numpy-2.2.4 setuptools-78.1.0
# pip install --upgrade "numpy<2"
# pip install numpy==2.1.1

class ConeDetectionNode(Node):

    def __init__(self):
        super().__init__("camera_cone_node")

        # Declare and get model path parameter
        self.declare_parameter("model_path", "yolov8n.pt")  # default to yolov8n.pt it none found
        self.model_path = self.get_parameter("model_path").value

        # Load YOLO model with custom weights
        self.device = "cuda" if torch.cuda.is_available() else "cpu"  
        self.model = YOLO(self.model_path).to(self.device)  # load model onto the device

        # Obtain ROS parameters for YOLO mode
        self.declare_parameter("yolo_conf", value=0.5)
        self.declare_parameter("yolo_iou", value=0.3)
        self.conf = self.get_parameter("yolo_conf").value
        self.iou = self.get_parameter("yolo_iou").value

        # Constructs queue for frame generation
        self.frame_queue = queue.Queue(maxsize=10)

        # Instantiate ROS callbacks for adding image to stream generation
        self.create_subscription(Image, "image/raw", self.callback, 10)
        self.create_subscription(Image, "image/depth", self.depth_callback, 5)
        self.cone_image_publisher = self.create_publisher(Image, "image/cones", 5)
        self.bridge = CvBridge()

        # Start tracker thread
        self.object_tracking_t = threading.Thread(target=self.object_tracking, daemon=True)
        self.object_tracking_t.start()

    def callback(self, msg: Image) -> None:
        """
        ROS2 subscription callback. Convert the ROS Image to OpenCV (numpy) and enqueue it.
        If the queue is full, drop the oldest frame to make room (so we stay 'real-time').
        
        Args:
            msg: sensor_msgs.msg.Image
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        try:
            self.frame_queue.put(frame, timeout=3.0)
        except queue.Full:
            self.frame_queue.get_nowait()
            self.frame_queue.put_nowait(frame)
        
        return
    
    def depth_callback(self, msg: Image) -> None:
        """
        ROS2 Subscription callback. Applies the concurrent rgbd object bounding box onto a integer
        depth mask. Obtains the average over the bounding box, and calculates the depth of
        objects.
        """



        return
    
    def frame_generator(self) -> Generator[np.ndarray, None, None]:
        """
        Python generator func that yields OpenCV frames pulled from self.frame_queue.
        model.track() can treat this as a 'video stream'.

        Note: Function blocks the node until video frames are available

        Args:
            None

        Returns:
            frame: Generator[np.ndarray, None, None]
        """
        while rclpy.ok():
            frame = self.frame_queue.get()   # blocking get
            if frame is None:
                break
            yield frame
    
    def object_tracking(self) -> None:
        """
        Launch YOLOv8's tracker on a continuous stream of frames coming from frame_generator().
        The .track(...) call will loop internally over frames until shutdown.

        Args:
            None
        
         Returns:
            None
        """
        results = self.model.track(
            source=self.frame_generator(),
            tracker="bytetrack.yaml",
            persist=True,
            stream=True,
            conf=self.conf,
            iou=self.iou,
            verbose=True
        )

        # Obtain the boxes and track IDs
        if ((results.boxes and results.boxes.is_track) and 
            self.cone_image_publisher.get_subscription_count()):
            boxes = results.boxes.xywh.cpu()
            track_ids = results.boxes.id.int().cpu().tolist()

            print(boxes)
            print(track_ids)

            for frame_result in zip(boxes, track_ids):
                # Publish the results through ROS subscribers and publishers

                continue
            
        return
    
def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()