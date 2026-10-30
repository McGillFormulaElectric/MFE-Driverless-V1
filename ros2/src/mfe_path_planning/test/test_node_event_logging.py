"""
Regression test for the planner node's event logging against a real rclpy node.

The planner once passed throttle_duration_sec values through one rclpy logger call site. rclpy
rejects that when the values differ between calls ("Requested logging filters cannot be changed
between calls"), which killed path_planner_node on its first cone callback in the sim.

This test cannot run in the pytest process: conftest.py replaces rclpy with stubs. It runs a
child Python process that imports the real rclpy and calls PathPlannerNode._log_events on a real
node, twice, with throttled and unthrottled events mixed. PLANNER_PKG_DIR (optional) points the
child at another copy of the mfe_path_planning package, for checking the old code.
"""

import os
import subprocess
import sys

import pytest

PKG_DIR = os.environ.get('PLANNER_PKG_DIR', os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

CHILD = r'''
import sys
sys.path.insert(0, sys.argv[1])
import rclpy
import mfe_path_planning.path_planner_node as ppn

rclpy.init()
node = ppn.PathPlannerNode()
events = [
    ('info', 'unthrottled event', {}),
    ('info', 'throttled five seconds', {'throttle_duration_sec': 5.0}),
    ('warn', 'throttled one second', {'throttle_duration_sec': 1.0}),
    ('info', 'throttled five seconds, other value', {'throttle_duration_sec': 2.0}),
]
node._log_events(events)
node._log_events(events)
node.destroy_node()
rclpy.shutdown()
print('LOG_EVENTS_OK')
'''


def _rclpy_importable():
    result = subprocess.run([sys.executable, '-c', 'import rclpy'], capture_output=True, timeout=120)
    return result.returncode == 0


@pytest.mark.skipif(not _rclpy_importable(), reason='rclpy is not importable in this environment')
def test_log_events_mixed_throttle_does_not_raise():
    result = subprocess.run([sys.executable, '-c', CHILD, PKG_DIR], capture_output=True, text=True, timeout=300)
    output = result.stdout + result.stderr
    assert result.returncode == 0, output[-4000:]
    assert 'LOG_EVENTS_OK' in result.stdout, output[-4000:]
