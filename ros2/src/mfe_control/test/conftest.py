"""
Shared ROS2 mocks for mfe_control tests.
Imported automatically by pytest before any test in this directory.
"""

import sys
import types


def _install_ros_mocks():
    stubs = [
        'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
        'nav_msgs', 'nav_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg',
        'std_msgs', 'std_msgs.msg',
        'sensor_msgs', 'sensor_msgs.msg',
        'fs_msgs', 'fs_msgs.msg',
        'mfe_msgs', 'mfe_msgs.msg',
        'tf2_ros',
        # osqp intentionally NOT stubbed: its ImportError sets _HAS_OSQP=False,
        # which triggers the SLSQP fallback path in mpc_lateral_node.
    ]
    # Only stub scipy if not installed — let real scipy work when available
    try:
        import scipy  # noqa: F401
    except ImportError:
        stubs += ['scipy', 'scipy.optimize', 'scipy.sparse']
    for mod in stubs:
        if mod not in sys.modules:
            sys.modules[mod] = types.ModuleType(mod)

    class _QoS:
        RELIABLE = BEST_EFFORT = KEEP_LAST = 1
        def __init__(self, **kw): pass
    for attr in ['QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy']:
        setattr(sys.modules['rclpy.qos'], attr, _QoS)

    class _Node:
        def __init__(self, *a, **kw): pass
        def declare_parameter(self, name, default=None): pass
        def get_parameter(self, name):
            defaults = {
                'wheelbase': 1.56, 'max_steering_deg': 28.0,
                'horizon': 10, 'dt': 0.05,
                'weight_crosstrack': 10.0, 'weight_heading': 5.0,
                'weight_steer': 1.0, 'weight_terminal': 10.0,
                'delta_rate_max_deg': 5.0, 'v_min_active': 0.5,
                'v_model_floor': 1.0,
                'steer_pt2_tau': 0.05, 'steer_pt2_zeta': 0.7,
                'speed_kp': 0.5, 'speed_ki': 0.1,
                'K_v': 0.5, 'ld_min': 0.5, 'ld_max': 15.0,
                'lookahead_distance': 5.0, 'max_speed': 10.0,
                'max_lateral_accel': 8.0, 'max_deceleration': 10.0,
                'lookahead_waypoints': 40, 'speed_reduction_factor': 0.3,
                'car_half_length': 1.5, 'car_half_width': 0.8,
                'cone_hit_radius_m': 0.5,
                'lidar_timeout_ms': 200, 'gps_jump_threshold_m': 2.0,
                'gps_cov_threshold': 25.0, 'control_latency_threshold_ms': 100,
            }
            val = defaults.get(name, 0.0)
            return type('P', (), {'value': val})()
        def create_publisher(self, *a, **kw): return None
        def create_subscription(self, *a, **kw): return None
        def create_timer(self, *a, **kw): return None
        def get_logger(self):
            return type('L', (), {
                'info':  lambda s, *a, **kw: None,
                'warn':  lambda s, *a, **kw: None,
                'error': lambda s, *a, **kw: None,
            })()
        def get_clock(self):
            return type('C', (), {
                'now': lambda s: type('T', (), {'to_msg': lambda s: None})()
            })()
    sys.modules['rclpy.node'].Node = _Node

    # tf2_ros stubs
    class _TFEx(Exception): pass
    class _Buf:
        def __init__(self, *a, **k): pass
    sys.modules['tf2_ros'].TransformException = _TFEx
    sys.modules['tf2_ros'].Buffer             = _Buf
    sys.modules['tf2_ros'].TransformListener  = type('TFL', (), {'__init__': lambda s,*a,**k: None})

    # ControlCommand stub
    class _CC:
        def __init__(self):
            self.throttle = 0.0; self.brake = 0.0; self.steering = 0.0
            self.header = type('H', (), {'stamp': None, 'frame_id': ''})()
    sys.modules['fs_msgs.msg'].ControlCommand = _CC

    # std_msgs stubs
    class _Header:
        def __init__(self): self.stamp = None; self.frame_id = ''
    sys.modules['std_msgs.msg'].Bool             = type('Bool',   (), {'data': False})
    sys.modules['std_msgs.msg'].Float64MultiArray = type('F64MA', (), {'data': []})
    sys.modules['std_msgs.msg'].String           = type('String', (), {'data': ''})
    sys.modules['std_msgs.msg'].Header           = _Header
    sys.modules['std_msgs.msg'].Int32            = type('Int32',  (), {'data': 0})

    # sensor_msgs stubs
    sys.modules['sensor_msgs.msg'].PointCloud2 = type('PointCloud2', (), {
        'fields': [], 'data': b'', 'width': 0, 'height': 0, 'point_step': 0
    })
    sys.modules['sensor_msgs.msg'].PointField = type('PointField', (), {})
    sys.modules['sensor_msgs.msg'].NavSatFix  = type('NavSatFix',  (), {
        'COVARIANCE_TYPE_UNKNOWN': 0,
        'position_covariance': [0]*9,
        'position_covariance_type': 0,
    })

    # scipy.sparse stub (needed for OSQP path in mpc_lateral_node)
    try:
        import scipy.sparse as _sp
        if not hasattr(_sp, 'triu'):
            _sp.triu = lambda x, **kw: x
    except ImportError:
        sys.modules['scipy.sparse'].triu = lambda x, **kw: x
        sys.modules['scipy.sparse'].eye  = lambda n, **kw: None
        sys.modules['scipy.sparse'].block_diag = lambda *a, **kw: None
        sys.modules['scipy.sparse'].vstack     = lambda *a, **kw: None

    # nav_msgs / geometry_msgs stubs
    for cls in ['Odometry', 'Path']:
        setattr(sys.modules['nav_msgs.msg'], cls, type(cls, (), {}))
    for cls in ['PoseStamped', 'Pose', 'Point', 'Quaternion', 'Twist', 'Vector3']:
        setattr(sys.modules['geometry_msgs.msg'], cls, type(cls, (), {}))

    # mfe_msgs/Cone stub
    class _Cone:
        BLUE = 0; YELLOW = 1; ORANGE_BIG = 2; ORANGE_SMALL = 3; UNKNOWN = 4
    sys.modules['mfe_msgs.msg'].Cone  = _Cone
    sys.modules['mfe_msgs.msg'].Track = type('Track', (), {'track': []})


_install_ros_mocks()
