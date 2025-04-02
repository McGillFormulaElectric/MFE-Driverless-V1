import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs import Float64
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import numpy as np
import logging

from carmaker_interface import CarMaker, CameraRSDR, LidarRSDR
from carmaker_interface import Quantity

import cv2

class CarMakerBridgeNode(Node):
    def __init__(self): 
        super().__init__("carmaker_bridge_node")

        self.declare_parameter("ip_address", "localhost")
        self.declare_parameter("port", 10000)
        self.declare_parameter("log_level", "INFO")

        self.ip_address = self.get_parameter("ip_address").value
        self.port = self.get_parameter("port").value
        self.log_level_param = self.get_parameter("log_level").value

        match (self.log_level):
            case "DEBUG":
                self.log_level = logging.DEBUG
            case "INFO":
                self.log_level = logging.INFO
            case "ERROR":
                self.log_level = logging.ERROR
            case _:
                self.log_level = logging.INFO 
        
        # --- CarMaker TCP interface for reading/writing simulation quantities ---
        self.carmaker = CarMaker(ip=self.ip_address, port=self.port, log_level=self.log_level)

        # List all topics to subscribe 
        self.speed_quantity = Quantity("Car.v", Quantity.FLOAT) # TODO: generalize
        self.speed_quantity = -1.0
        self.carmaker.subscribe(self.speed_quantity)

        self.camera = CameraRSDR(ip=self.ip_address, port=self.port, log_level=self.log_level)
        # Potentially consider adding subscription to quantity here
        self.lidar = LidarRSDR(ip=self.ip_address, port=self.port, log_level=self.log_level)
        # Potentially consider adding subscription to quantity here

        self.carmaker.connect()
        self.camera.connect()
        self.lidar.connect()

        self.cv_bridge = CvBridge()

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  
            depth=10,                                  # keep last 10 messages
            reliability=ReliabilityPolicy.RELIABLE,     # reliable
        )

        # --- Publishers that write data --- 
        # 1) Speed as Float64
        self.speed_pub = self.create_publisher(Float64, '/sim/speed', qos_profile)
        # 2) Camera images
        self.camera_pub = self.create_publisher(Image, '/camera/image/raw', qos_profile)
        # 3) LiDAR PointCloud2
        self.lidar_pub = self.create_publisher(PointCloud2, 'lidar_points', qos_profile)

        # --- Timers / Periodic Callbacks --- 
        # read CarMaker data at 20 Hz
        self.read_carmaker_timer = self.create_timer(0.05, self.read_carmaker_data)
        # read camera frames at 10 Hz (adjust as needed)
        self.read_camera_timer = self.create_timer(0.1, self.read_camera_data)
        # read LiDAR at 10 Hz (adjust as needed)
        self.read_lidar_timer = self.create_timer(0.1, self.read_lidar_data)

        self.get_logger().info("CarMaker Bridge Node has been initialized.")


    def read_carmaker_data(self):
        self.carmaker.read()

        msg = Float64()
        msg.data = self.speed_quantity.data
        self.speed_pub.publish(msg)

        return None

    def read_camera_data(self):
        frame = self.camera.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        self.camera_pub.publish(msg)

        return None

    def read_lidar_data(self):
        self.lidar.read()
        # TODO: Add intermediate logic

        msg = PointCloud2()

        self.lidar_pub.publish(msg)

        return None

    def write_carmaker_data(self):
        # Call DVA Write

        # Call DVA release

        return None

    def write_camera_data(self):
        # Call DVA Write

        # Call DVA Release

        return None

    def write_lidar_data(self):
        # Call DVA Write

        # Call DVA release

        return None
    
    
def main(args=None):
    rclpy.init(args=args)

    node = CarMakerBridgeNode() # Instantiates CarMakerBridge 

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Shutting down CarMaker node...")

    node.destroy_node()
    rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()