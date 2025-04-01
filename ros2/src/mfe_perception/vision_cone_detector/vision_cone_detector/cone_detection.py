#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import torch
import pandas as pd
import numpy as np

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge


# yolov5
MODEL_PATH = "yolov5s.pt" # TODO the path to the weights
model = torch.hub.load('ultralytics/yolov5', 'custom', path=MODEL_PATH)
model.conf = 0.5  # TODO confidence threshold


class CameraConeNode(Node):

    def __init__(self):
        super().__init__("camera_cone_node")

        self.create_subscription(Image, "camera/image/raw", self.callback, 10)

        # FOR DEBUGGING
        self.cone_image_publisher = self.create_publisher(Image, "camera/image/cones", 10)
        
        # to the mapping/get depth? one
        self.cone_detection_publisher = self.create_publisher(Detection2DArray, "camera/detection/cones", 10)
        
        self.bridge = CvBridge()

        return


    def callback(self, msg):

        # convert from ROS image to opencv format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # run yolov5
        results = model(frame)
        detections = results.pd().xyxy[0]  # Pandas DataFrame with bbox results

        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header  

        # add bounding boxes
        for _, row in detections.iterrows():
            x1, y1, x2, y2, conf, cls, label = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax']), row['confidence'], int(row['class']), row['name']

            # draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # create Detection2D msg
            detection_msg = Detection2D()
            detection_msg.bbox.center.x = (x1 + x2) / 2.0
            detection_msg.bbox.center.y = (y1 + y2) / 2.0
            detection_msg.bbox.size_x = x2 - x1
            detection_msg.bbox.size_y = y2 - y1

            # ObjectHypothesisWithPose (class + confidence)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = cls
            hypothesis.score = float(conf)
            detection_msg.results.append(hypothesis)

            detection_array_msg.detections.append(detection_msg)

        # publish
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.cone_image_publisher.publish(image_msg)
        self.cone_detection_publisher.publish(detection_array_msg)

        return
    
def main(args=None):
    rclpy.init(args=args)

    node = CameraConeNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()