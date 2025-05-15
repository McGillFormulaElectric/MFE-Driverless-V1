import numpy as np

import rclpy
from rclpy import Node

class OdomTransformerNode(Node):
    """
    Internal Odometry Data Transformation Node.

    Formats raw data obtained from real sensors / simulation sensors into usable data by the SLAM
    pipeline.

    Subscriptions:
        imu_topic (String) based on parameter
        gps_topic (String) based on parameter 
        lidar_topic (String) based on parameter
    
    Publishers:
        imu/odometry
        gps/odometry
        lidar/odometry
    """

    def __init__(self): 
        self.declare_parameter("imu_topic", value="imu/data")
        self.imu_topic_name = self.get_parameter("imu_topic").value
        
        self.declare_parameter("gps_topic", value="gps/data")
        self.gps_topic_name = self.get_parameter("gps_topic").value

        self.create_subscription(
            
        )

    def imu_callback(self, msg):

        return
    
    def gps_callback(self, msg): 

        return 
    
    def lidar_callback(self, msg):

        return