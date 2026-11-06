"""ROS message helpers for planner paths."""
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def path_from_xy(xy: np.ndarray, frame_id: str, stamp) -> Path:
    """Nx2 array of (x, y) -> nav_msgs/Path with identity orientation."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for point in xy:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = stamp
        ps.pose.position.x = float(point[0])
        ps.pose.position.y = float(point[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path
