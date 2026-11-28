"""
Shared pytest fixtures and conditional ROS 2 mocks for mfe_path_planning tests.

Mocks rclpy and ROS message types only when they are not installed so
algorithm-only tests run outside a ROS 2 environment while real libraries
(e.g., fsd_path_planning, scipy) are used whenever available.
"""

import importlib.util
import os
from pathlib import Path
import sys
import types

import pytest


def _ensure_local_packages_on_path():
    """Add mfe_path_planning and mfe_common source roots when not installed."""
    test_dir = Path(__file__).resolve().parent
    pkg_root = test_dir.parent
    src_root = pkg_root.parent
    for candidate in (pkg_root, src_root / 'mfe_common'):
        cand_str = str(candidate)
        if candidate.is_dir() and cand_str not in sys.path:
            sys.path.insert(0, cand_str)


def _has_real_module(mod_name: str) -> bool:
    """Return True if a real (non-stub) module can be imported."""
    top = mod_name.split('.')[0]
    if top in sys.modules:
        return getattr(sys.modules[top], '__file__', None) is not None
    try:
        return importlib.util.find_spec(top) is not None
    except (ImportError, ValueError):
        return False


def _install_ros_mocks():
    """Install lightweight stubs only for unavailable ROS 2 / optional modules."""
    stubs = [
        'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
        'nav_msgs', 'nav_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg',
        'sensor_msgs', 'sensor_msgs.msg',
        'std_msgs', 'std_msgs.msg',
        'mfe_msgs', 'mfe_msgs.msg',
        'eufs_msgs', 'eufs_msgs.msg',
        'fs_msgs', 'fs_msgs.msg',
        'fsd_path_planning', 'ft_fsd_path_planning',
        'tf2_ros',
        'diagnostic_msgs', 'diagnostic_msgs.msg',
    ]
    installed = set()
    for mod in stubs:
        if mod not in sys.modules and not _has_real_module(mod):
            sys.modules[mod] = types.ModuleType(mod)
            installed.add(mod)

    if 'rclpy.qos' in installed:
        class _QoS:
            RELIABLE = BEST_EFFORT = KEEP_LAST = TRANSIENT_LOCAL = 1

            def __init__(self, **kw):
                pass

        for attr in ('QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy', 'DurabilityPolicy'):
            setattr(sys.modules['rclpy.qos'], attr, _QoS)

    if 'rclpy.node' in installed:
        class _Node:
            def __init__(self, *a, **kw):
                pass

            def declare_parameter(self, *a, **kw):
                pass

            def get_parameter(self, name):
                return type('P', (), {'value': None})()

            def create_publisher(self, *a, **kw):
                return None

            def create_subscription(self, *a, **kw):
                return None

            def create_timer(self, *a, **kw):
                return None

            def get_logger(self):
                return type('L', (), {
                    'info': lambda s, *a, **kw: None,
                    'warn': lambda s, *a, **kw: None,
                    'error': lambda s, *a, **kw: None,
                })()

            def get_clock(self):
                return type('C', (), {
                    'now': lambda s: type('T', (), {'to_msg': lambda s: None})()
                })()

        sys.modules['rclpy.node'].Node = _Node

    if 'nav_msgs.msg' in installed:
        for cls in ('Odometry', 'Path'):
            setattr(sys.modules['nav_msgs.msg'], cls, type(cls, (), {}))

    if 'geometry_msgs.msg' in installed:
        for cls in ('PoseStamped', 'Pose', 'Point', 'Quaternion', 'Twist', 'Vector3'):
            setattr(sys.modules['geometry_msgs.msg'], cls, type(cls, (), {}))

    if 'mfe_msgs.msg' in installed:
        class _Cone:
            BLUE = 0
            YELLOW = 1
            ORANGE_BIG = 2
            ORANGE_SMALL = 3
            UNKNOWN = 4

            def __init__(self):
                self.color = _Cone.UNKNOWN
                self.location = type('P', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
                self.header = type('H', (), {'stamp': None, 'frame_id': 'map'})()

        class _Track:
            def __init__(self):
                self.track = []

        sys.modules['mfe_msgs.msg'].Cone = _Cone
        sys.modules['mfe_msgs.msg'].Track = _Track

    if 'std_msgs.msg' in installed:
        class _F64MA:
            def __init__(self):
                self.data = []

        class _Header:
            def __init__(self):
                self.stamp = None
                self.frame_id = ''

        sys.modules['std_msgs.msg'].Float64MultiArray = _F64MA
        sys.modules['std_msgs.msg'].Float32 = type('Float32', (), {'data': 0.0})
        sys.modules['std_msgs.msg'].Bool = type('Bool', (), {'data': False})
        sys.modules['std_msgs.msg'].String = type('String', (), {'data': ''})
        sys.modules['std_msgs.msg'].Int32 = type('Int32', (), {'data': 0})
        sys.modules['std_msgs.msg'].Header = _Header

    if 'sensor_msgs.msg' in installed:
        sys.modules['sensor_msgs.msg'].PointCloud2 = type('PointCloud2', (), {
            'fields': [], 'data': b'', 'width': 0, 'height': 0, 'point_step': 0
        })
        sys.modules['sensor_msgs.msg'].PointField = type('PointField', (), {})
        sys.modules['sensor_msgs.msg'].NavSatFix = type('NavSatFix', (), {
            'COVARIANCE_TYPE_UNKNOWN': 0, 'position_covariance': [0] * 9,
            'position_covariance_type': 0
        })

    if 'tf2_ros' in installed:
        class _TFException(Exception):
            pass

        class _Buffer:
            def __init__(self, *a, **k):
                pass

            def lookup_transform(self, *a, **k):
                raise _TFException('stub')

        class _TFListener:
            def __init__(self, *a, **k):
                pass

        sys.modules['tf2_ros'].TransformException = _TFException
        sys.modules['tf2_ros'].Buffer = _Buffer
        sys.modules['tf2_ros'].TransformListener = _TFListener

    if not _has_real_module('scipy'):
        import numpy as np
        _opt = types.ModuleType('scipy.optimize')

        def _lsa(cost):
            cost = np.array(cost)
            if cost.size == 0:
                return np.array([], dtype=int), np.array([], dtype=int)
            r = np.arange(min(cost.shape))
            c = np.argmin(cost[:min(cost.shape)], axis=1)
            return r, c

        _opt.linear_sum_assignment = _lsa
        sys.modules['scipy'] = types.ModuleType('scipy')
        sys.modules['scipy.optimize'] = _opt


_ensure_local_packages_on_path()
_install_ros_mocks()


def pytest_addoption(parser):
    """Register CLI options for benchmark report export."""
    parser.addoption(
        '--report-out',
        action='store',
        default=os.environ.get('MFE_PLANNING_REPORT_OUT', ''),
        help='Optional JSON path to write structured benchmark metrics.',
    )


def pytest_configure(config):
    """Register custom markers for path planning tests."""
    config.addinivalue_line('markers', 'slow: long-running matrix or stress tests')
    config.addinivalue_line('markers', 'sim: requires live Gazebo simulation')
    config.addinivalue_line('markers', 'fuzz: procedural random-track fuzzing tests')


@pytest.fixture(scope='session')
def centerline_fixtures_dir() -> Path:
    """Return directory containing committed ground truth centerline CSVs."""
    return Path(__file__).resolve().parent / 'fixtures' / 'centerlines'


@pytest.fixture(scope='session')
def eufs_tracks_dir() -> Path:
    """Return EUFS track CSV directory or skip if unavailable."""
    from mfe_path_planning.eval.tracks import find_eufs_tracks_dir
    tracks_dir = find_eufs_tracks_dir()
    if tracks_dir is None:
        pytest.skip('EUFS track CSV directory not found (set EUFS_MASTER)')
    return tracks_dir


@pytest.fixture(scope='session')
def report_out_path(request) -> str:
    """Return configured JSON report output path (empty string if disabled)."""
    return request.config.getoption('--report-out') or ''
