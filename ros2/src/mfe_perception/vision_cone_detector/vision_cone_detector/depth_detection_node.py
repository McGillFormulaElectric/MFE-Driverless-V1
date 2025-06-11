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

class DepthDetectionNode(Node):

    def __init__(self):
        super().__init__("depth_detection_node")

        self.cone_depth_publisher

        
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
            try:
                frame = self.frame_queue.get()   # blocking get 
            except queue.Empty:
                continue
            if frame is None:
                break

            yield frame

    def object_predicting(self) -> None:
        """
        Launches YOLO's prediction model on a continuous stream of frames coming 
        from frame_generator().
        """
    
        for frame in self.frame_generator():

            # run object detection
            detections = self.model(frame)

            annotated_frame = detections[0].plot()

            # render bounding boxes
            self.render_detection(frame, detections)

            # publish
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.cone_image_publisher.publish(image_msg)
            #self.cone_detection_publisher()

        return
    
    def object_tracking(self) -> None:
        """
        Launch YOLO's tracker model on a continuous stream of frames coming from frame_generator().
        The .track(...) call will loop internally over frames until shutdown.

        Args:
            None
        
         Returns:
            None
        """

        for frame in self.frame_generator():

            results = self.model.track(
                source=frame,
                tracker="bytetrack.yaml",
                persist=True,
                stream=True,
                conf=self.conf,
                iou=self.iou,
                verbose=True
            )

            for result in results:
                annotated_frame = result.plot()
                cv2.imshow("YOLO11 Tracking", annotated_frame)

                if ((result.boxes and result.boxes.is_track) and 
                    self.cone_image_publisher.get_subscription_count()):
                    
                    boxes = result.boxes.xywh.cpu()
                    track_ids = result.boxes.id.int().cpu().tolist()

                    for frame_result in zip(boxes, track_ids):
                        # Publish the results through ROS subscribers and publishers
                        continue

    def render_detection(self, frame, detections):
        """
        Renders bounding boxes around the objects detected on the frames for displaying purposes
        Args:
            frame (numpy.ndarray) : image in cv2-readable type from the camera
            detections (pd.DataFrame) : dataframe of info of objects detected by model
        """

        for r in detections:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls

                # render
                cv2.rectangle(frame, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
                # cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return
        
def main(args=None):
    rclpy.init(args=args)
    node = DepthDetectionNode

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()