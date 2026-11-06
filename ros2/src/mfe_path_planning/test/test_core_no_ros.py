"""
Boundary test: the planner core must import with rclpy and the ROS message packages blocked, and.

its sources must not name them. The node (path_planner_node.py) is the only planner module allowed to use ROS.
"""
import importlib
import os
import pkgutil
import re
import sys

import pytest

import mfe_path_planning.core as core_pkg

BLOCKED = ('rclpy', 'nav_msgs', 'std_msgs', 'sensor_msgs', 'geometry_msgs', 'mfe_msgs', 'fs_msgs', 'eufs_msgs',
           'tf2_ros', 'builtin_interfaces', 'fsd_path_planning', 'ft_fsd_path_planning')
CORE_DIR = os.path.dirname(core_pkg.__file__)
FORBIDDEN = re.compile(r'^\s*(from|import)\s+(rclpy|nav_msgs|std_msgs|sensor_msgs|geometry_msgs|mfe_msgs|fs_msgs|'
                       r'eufs_msgs|tf2_ros|builtin_interfaces)\b', re.MULTILINE)


def _core_module_names():
    return sorted(m.name for m in pkgutil.iter_modules(core_pkg.__path__))


def test_core_has_modules():
    names = _core_module_names()
    for expected in ('accel', 'boundary_tracer', 'cone_map', 'colour', 'finish', 'jump_gate', 'lap', 'params',
                     'paths', 'planning', 'raceline', 'raceline_runner', 'skidpad'):
        assert expected in names, expected


def test_core_sources_name_no_ros_packages():
    offenders = []
    for root, _, files in os.walk(CORE_DIR):
        for f in files:
            if f.endswith('.py'):
                path = os.path.join(root, f)
                with open(path, encoding='utf-8') as fh:
                    if FORBIDDEN.search(fh.read()):
                        offenders.append(path)
    assert not offenders, f'ROS imports in core: {offenders}'


def test_core_modules_import_with_ros_blocked(monkeypatch):
    for key in list(sys.modules):
        if key.startswith('mfe_path_planning.core'):
            monkeypatch.delitem(sys.modules, key, raising=False)
    for name in BLOCKED:
        monkeypatch.setitem(sys.modules, name, None)
    for name in _core_module_names():
        importlib.import_module(f'mfe_path_planning.core.{name}')
    with pytest.raises(ImportError):
        importlib.import_module('rclpy')
