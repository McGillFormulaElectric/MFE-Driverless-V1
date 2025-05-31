#!/usr/bin/env python3

import threading
import collections
import time

import rclpy

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FrameGenerator():
    def __init__(self):
        """
        A Python generator that yields the newest frame as soon as it arrives.
        If no frame is available yet, it waits in a tight loop (or a short sleep).
        """

        self.frame_queue = collections.deque(maxlen=1)  # only care about 1 frame

        self.bridge = CvBridge()
        
    def add_image(self, image_msg: Image) -> None:
        frame_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        self.frame_queue.append(frame_bgr)

    def frame_generator(self) -> :
        """Generates the frame iterator"""

        while rclpy.ok():
            if (len(self.frame_queue) == 0):
                time.sleep(0.05)
                continue
        
            frame_bgr = self.frame_queue.pop()
            roi = frame_bgr

            frame_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
            
            yield frame_rgb, frame_bgr     # use yield to generate continuous iterator