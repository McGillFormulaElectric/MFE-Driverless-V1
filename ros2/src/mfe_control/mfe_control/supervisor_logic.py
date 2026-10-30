"""Pure supervisor helpers (no ROS): the GPS great-circle distance and the cone oriented-bounding-box hit test."""
import math

import numpy as np

EARTH_RADIUS_M = 6_371_000.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Approximate great-circle distance in metres between two WGS-84 points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def obb_local(cone_xy, car_x: float, car_y: float, car_yaw: float):
    """Cones (Nx2, map frame) in the car frame: (local_x forward, local_y left).

    Car-local axes (ENU map -> car body):
        local_x =  dx*cos(yaw) + dy*sin(yaw)   (forward)
        local_y = -dx*sin(yaw) + dy*cos(yaw)   (left)
    """
    dx = cone_xy[:, 0] - car_x
    dy = cone_xy[:, 1] - car_y
    cos_y = math.cos(car_yaw)
    sin_y = math.sin(car_yaw)
    local_x = dx * cos_y + dy * sin_y
    local_y = -dx * sin_y + dy * cos_y
    return local_x, local_y


def obb_hits(cone_xy, car_x: float, car_y: float, car_yaw: float, half_length: float, half_width: float):
    """Oriented-bounding-box cone test. Returns (inside, local_x, local_y): inside is True for a cone with
    |local_x| < half_length and |local_y| < half_width (a side-swipe or a cone under the car)."""
    local_x, local_y = obb_local(cone_xy, car_x, car_y, car_yaw)
    inside = (np.abs(local_x) < half_length) & (np.abs(local_y) < half_width)
    return inside, local_x, local_y
