"""
Shared fixtures for mfe_perception rosbag-based tests.

The rosbag reader (rosbags library) works without ROS2 installed.
Tests skip automatically when:
  - rosbags library not installed
  - bag file is missing or empty (< 1 KB)
"""

import os
import pytest

BAG_PATH = os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', '..', '..',   # repo root
    'rosbags', 'amz-vision-2017'
)
BAG_PATH = os.path.normpath(BAG_PATH)
BAG_DB   = os.path.join(BAG_PATH, 'amz-vision-2017.db3')

# Minimum bag size to be considered real (empty stub = 134 bytes)
_MIN_BAG_BYTES = 1024


def bag_available() -> bool:
    if not os.path.exists(BAG_DB):
        return False
    if os.path.getsize(BAG_DB) < _MIN_BAG_BYTES:
        return False
    try:
        from rosbags.rosbag2 import Reader
        with Reader(BAG_PATH) as r:
            return r.message_count > 0
    except Exception:
        return False


BAG_SKIP = pytest.mark.skipif(
    not bag_available(),
    reason=(
        f'Rosbag not available or empty at {BAG_PATH}. '
        'Record a bag with: ros2 bag record --storage-id mcap '
        '/velodyne_points /imu /gps /wheel_rpm /optical_speed_sensor '
        '-o rosbags/amz-vision-2017'
    )
)


@pytest.fixture(scope='session')
def bag_reader():
    """Session-scoped rosbag Reader — opened once, shared across all tests."""
    from rosbags.rosbag2 import Reader
    with Reader(BAG_PATH) as r:
        yield r
