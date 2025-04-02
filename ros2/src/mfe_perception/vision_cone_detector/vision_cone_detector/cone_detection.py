#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO


import cv2
import torch
import pandas as pd
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point


# installs numpy-2.2.4 setuptools-78.1.0
# pip install --upgrade "numpy<2"
# pip install numpy==2.1.1



class CameraConeNode(Node):

    def __init__(self):
        super().__init__("camera_cone_node")


        # Declare and get model path parameter
        self.declare_parameter("model_path", "yolov8n.pt")  # Default to yolov8n.pt
        self.model_path = self.get_parameter("model_path").value

        # Load the model with custom weights
        self.device = "cuda" if torch.cuda.is_available() else "cpu"  # Use GPU if available
        self.model = YOLO(self.model_path).to(self.device)  # Load model onto the device

        # Set confidence threshold
        self.model.conf = 0.5 

        self.create_subscription(Image, "image/raw", self.callback, 10)

        self.cone_image_publisher = self.create_publisher(Image, "image/cones", 10) # show
        #self.cone_detection_publisher = self.create_publisher(ConeBoxes, "")
        
        self.bridge = CvBridge()

        return

    def object_detection(self, frame):
        """
        Runs object detection on frame
        """

        results = self.model.predict(frame, verbose=True)
        detections = pd.DataFrame(results[0].boxes.data.tolist(), columns=['x', 'y', 'width', 'height', 'confidence', 'class'])


        return detections
    

    def render_detection(self, frame, results):
        """
        Renders bounding boxes on the images for displaying purposes
        """

        # add bounding boxes

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls

                # draw box
                cv2.rectangle(frame, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
                # cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return
    

    def calc_depth(self):
        """
        Calculates depth of objects detected
        """



        return


    def callback(self, msg):

        # convert from ROS image to opencv format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # run object detection
        results = self.model(frame)

        # render bounding boxes
        self.render_detection(frame, results)

        
        # publish
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.cone_image_publisher.publish(image_msg)
        #self.cone_detection_publisher()

        return
    
def main(args=None):
    rclpy.init(args=args)

    node = CameraConeNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()