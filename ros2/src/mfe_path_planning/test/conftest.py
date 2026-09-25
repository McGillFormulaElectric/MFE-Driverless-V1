"""
Shared pytest fixtures and ROS2 mock for mfe_path_planning tests.

Mocks rclpy and all ROS message types so algorithm-only tests run
without a ROS2 environment (plain `pytest` or `colcon test`).
"""

import sys
import types


def _install_ros_mocks():
    """Install lightweight stubs for all ROS2 modules used by path_planner_node."""
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
    for mod in stubs:
        if mod not in sys.modules:
            sys.modules[mod] = types.ModuleType(mod)

    # rclpy.qos stubs
    class _QoS:
        RELIABLE = BEST_EFFORT = KEEP_LAST = TRANSIENT_LOCAL = 1
        def __init__(self, **kw): pass
    for attr in ['QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy', 'DurabilityPolicy']:
        setattr(sys.modules['rclpy.qos'], attr, _QoS)

    # rclpy.node stub
    class _Node:
        def __init__(self, *a, **kw): pass
        def declare_parameter(self, *a, **kw): pass
        def get_parameter(self, name):
            return type('P', (), {'value': None})()
        def create_publisher(self, *a, **kw): return None
        def create_subscription(self, *a, **kw): return None
        def create_timer(self, *a, **kw): return None
        def get_logger(self): return type('L', (), {
            'info': lambda s, *a, **kw: None,
            'warn': lambda s, *a, **kw: None,
            'error': lambda s, *a, **kw: None,
        })()
        def get_clock(self): return type('C', (), {
            'now': lambda s: type('T', (), {'to_msg': lambda s: None})()
        })()
    sys.modules['rclpy.node'].Node = _Node

    # nav_msgs / geometry_msgs stubs
    for cls in ['Odometry', 'Path']:
        setattr(sys.modules['nav_msgs.msg'], cls, type(cls, (), {}))
    for cls in ['PoseStamped', 'Pose', 'Point', 'Quaternion', 'Twist', 'Vector3']:
        setattr(sys.modules['geometry_msgs.msg'], cls, type(cls, (), {}))

    # Cone / Track stubs
    class _Cone:
        BLUE = 0; YELLOW = 1; ORANGE_BIG = 2; ORANGE_SMALL = 3; UNKNOWN = 4
        def __init__(self):
            self.color = _Cone.UNKNOWN
            self.location = type('P', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            self.header = type('H', (), {
                'stamp': None, 'frame_id': 'map'
            })()
    class _Track:
        def __init__(self): self.track = []
    sys.modules['mfe_msgs.msg'].Cone  = _Cone
    sys.modules['mfe_msgs.msg'].Track = _Track

    # std_msgs stubs
    class _F64MA:
        def __init__(self): self.data = []
    class _Header:
        def __init__(self): self.stamp = None; self.frame_id = ''
    sys.modules['std_msgs.msg'].Float64MultiArray = _F64MA
    sys.modules['std_msgs.msg'].Bool   = type('Bool',   (), {'data': False})
    sys.modules['std_msgs.msg'].String = type('String', (), {'data': ''})
    sys.modules['std_msgs.msg'].Int32  = type('Int32',  (), {'data': 0})
    sys.modules['std_msgs.msg'].Header = _Header

    # sensor_msgs stubs
    sys.modules['sensor_msgs.msg'].PointCloud2 = type('PointCloud2', (), {
        'fields': [], 'data': b'', 'width': 0, 'height': 0, 'point_step': 0
    })
    sys.modules['sensor_msgs.msg'].PointField = type('PointField', (), {})
    sys.modules['sensor_msgs.msg'].NavSatFix   = type('NavSatFix',  (), {
        'COVARIANCE_TYPE_UNKNOWN': 0, 'position_covariance': [0]*9,
        'position_covariance_type': 0
    })

    # tf2_ros stubs — TransformException and Buffer/TransformListener
    class _TFException(Exception): pass
    class _Buffer:
        def __init__(self, *a, **k): pass
        def lookup_transform(self, *a, **k): raise _TFException("stub")
    class _TFListener:
        def __init__(self, *a, **k): pass
    sys.modules['tf2_ros'].TransformException  = _TFException
    sys.modules['tf2_ros'].Buffer              = _Buffer
    sys.modules['tf2_ros'].TransformListener   = _TFListener

    # scipy stubs (Hungarian) — only stub if scipy isn't actually installed
    try:
        from scipy.optimize import linear_sum_assignment  # noqa: F401
    except ImportError:
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


_install_ros_mocks()
