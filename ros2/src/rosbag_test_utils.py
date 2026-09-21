"""
Shared utilities for rosbag-based tests.

Placed at ros2/src/ root so any test package can import it
without conftest namespace conflicts.

Usage in test files:
    import sys, os
    sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..')))
    from rosbag_test_utils import BAG_SKIP, BAG_PATH, bag_reader
"""

import os
import pytest

BAG_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'rosbags', 'amz-vision-2017'
))
BAG_DB = os.path.join(BAG_PATH, 'amz-vision-2017.db3')

_MIN_BAG_BYTES = 1024   # empty stub = 134 bytes


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
        f'Rosbag empty or missing at {BAG_PATH}. '
        'Record one with:\n'
        '  ros2 bag record --storage-id mcap '
        '/velodyne_points /imu /gps /wheel_rpm /optical_speed_sensor '
        '-o rosbags/amz-vision-2017'
    )
)


@pytest.fixture(scope='session')
def bag_reader():
    """Session-scoped Reader — opened once, shared across the whole test run."""
    from rosbags.rosbag2 import Reader
    with Reader(BAG_PATH) as r:
        yield r
