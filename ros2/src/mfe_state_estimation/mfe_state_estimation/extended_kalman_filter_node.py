import numpy as np

import rclpy
from rclpy import Node

from tf2_py import euler_

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class ExtendedKalmanFilterNode(Node):
    def __init__(self):
        super().__init__("ekf_node")

        self.declare_parameter("imu_frequency", value=20)
        self.declare_parameter("gps_frequency", value=2)
        self.imu_freq = self.get_parameter("imu_frequency").get_parameter_value().integer_value
        self.gps_freq = self.get_parameter("gps_frequency").get_parameter_value().integer_value

        # Configure necessary topics names to subscribe to
        self.declare_parameter("imu_topic", value="/imu")
        self.declare_parameter("gps_topic", value="/gps")

        self.imu_topic_name = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.gps_topic_name = self.get_parameter("gps_topic").get_parameter_value().string_value

        # Declare and configure covariance matrices (which indicate noise) in the EKF
        self.declare_parameter("var_imu_acc")
        self.declare_parameter("var_imu_w")
        self.declare_parameter("var_gps")

        self.var_imu_acc = self.get_parameter("var_imu_acc").get_parameter_value().double_array_value
        self.var_imu_w = self.get_parameter("var_imu_w").get_parameter_value().double_value

        self.var_gps_param = self.get_parameter("var_gps").get_parameter_value().double_array_value
        self.var_gps = np.array(self.var_gps_param)

        self.declare_parameter("output_topic", value="/ekf/output")
        self.output_topic_name = self.get_parameter("output_topic")

        self.create_subscription(
            Imu,
            self.imu_topic_name,
            self.imu_callback,
            self.imu_freq
        )
        self.create_subscription(
            NavSatFix,
            self.gps_topic_name,
            self.update,
            self.gps_freq
        )

        # self.odom_pub_ = self.create_publisher(

        # )

        self.get_logger().info(
            "Initialized EKF Node for the following topics: %s %s -> %s", 
            self.imu_topic_name,
            self.gps_topic_name,
            self.output_topic_name
        )

    def imu_callback(self, msg: Imu): 
        self.imu_msg = msg

        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y

        self.imu_w = msg.angular_velocity.z

        # TODO: Get 2D yaw from the IMU using Euler to quaternion transformation function
        self.imu_theta = 0

        self.predict()

    def predict(self):
        return

    def update(self):   
        return
    
def main(args=None):
    rclpy.init()
    ekf_node = ExtendedKalmanFilterNode()
    rclpy.spin(ekf_node)

    ekf_node.destroy_node()
    rclpy.shutdown()