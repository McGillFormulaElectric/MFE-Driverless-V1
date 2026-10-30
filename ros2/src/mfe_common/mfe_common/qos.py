"""QoS profiles used by the planner, control and validation nodes, named by what they carry.

The profiles are module-level constants, shared by every subscription or publisher that uses them. Their fields
match the inline QoSProfile(...) calls they replace, so DDS behaviour is unchanged. rclpy is imported here, so
this module needs a ROS 2 environment.
"""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Control, planning and validation messages (centerline, mission state, odometry, commands).
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

# Cone map topics: every /planning/cones message carries the whole map, so only the newest one matters.
LATEST_CONES_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

# Raw sensor streams (LiDAR, GPS, cone detections) where a late message is worth less than a dropped one.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

# Sensor streams read by the finish detector, with a shallower queue (depth 5).
SHALLOW_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

# One-shot state that a late subscriber must still receive (mission finished flag).
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
