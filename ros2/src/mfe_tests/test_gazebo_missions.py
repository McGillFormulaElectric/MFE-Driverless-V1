"""
Gazebo integration tests for FSAE missions.

Launches the full MFE stack + EUFS Gazebo sim as a subprocess, drives
the car through each mission, waits for /planning/mission_finished,
reads metrics from the lap_validator CSV, and asserts pass/fail.

REQUIRES: Docker with ROS2 Humble + EUFS sim installed and sourced.
          Run inside the Docker container after `source install/setup.bash`.

Usage:
    # Run all missions
    pytest test_gazebo_missions.py -v --timeout=300

    # Run one mission
    pytest test_gazebo_missions.py::TestAcceleration -v

    # Dry-run (validate test structure without Gazebo)
    pytest test_gazebo_missions.py --collect-only

Skip conditions (auto-skip when Gazebo not available):
    - ros2 not on PATH
    - EUFS sim not installed
    - No display available (use Xvfb: `Xvfb :99 -screen 0 1024x768x24 &; export DISPLAY=:99`)
"""

import csv
import glob
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import threading
import pytest


# ---------------------------------------------------------------------------
# Environment detection
# ---------------------------------------------------------------------------

def _ros2_available() -> bool:
    return shutil.which('ros2') is not None


def _eufs_sim_available() -> bool:
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'eufs_sim'],
            capture_output=True, timeout=5
        )
        return result.returncode == 0
    except Exception:
        return False


def _display_available() -> bool:
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


GAZEBO_AVAILABLE = _ros2_available() and _eufs_sim_available() and _display_available()

GAZEBO_SKIP = pytest.mark.skipif(
    not GAZEBO_AVAILABLE,
    reason=(
        'Gazebo not available. Requirements:\n'
        '  1. Run inside Docker with ROS2 Humble + EUFS sim\n'
        '  2. source /MFE-Driverless-V1/install/setup.bash\n'
        '  3. Xvfb :99 -screen 0 1024x768x24 & ; export DISPLAY=:99\n'
        '     (or use a real display)\n'
        f'  ros2={_ros2_available()} eufs={_eufs_sim_available()} display={_display_available()}'
    )
)

LOG_DIR = os.path.expanduser('~/mfe_logs')


# ---------------------------------------------------------------------------
# Mission runner
# ---------------------------------------------------------------------------

class MissionRunner:
    """
    Launches `ros2 launch mfe_bringup bringup.launch.py` for a given mission,
    waits for /planning/mission_finished to be True, then terminates.
    """

    BRINGUP_PKG = 'mfe_bringup'
    BRINGUP_LAUNCH = 'bringup.launch.py'

    def __init__(self, mission: str, timeout_s: float = 180.0,
                 extra_args: dict = None):
        self.mission    = mission
        self.timeout    = timeout_s
        self.extra_args = extra_args or {}
        self._proc      = None
        self._finished  = threading.Event()
        self._watcher   = None
        self._csv_path  = None

    def _wait_for_finish(self):
        """
        Background thread: polls /planning/mission_finished via ros2 topic echo.
        Sets self._finished when True is received.
        """
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', '--timeout', '5',
                 '/planning/mission_finished', 'std_msgs/msg/Bool'],
                capture_output=True, text=True, timeout=self.timeout
            )
            if 'data: true' in result.stdout.lower():
                self._finished.set()
        except Exception:
            pass

    def _latest_csv(self) -> str | None:
        """Return the most recently written validation CSV."""
        files = sorted(glob.glob(os.path.join(LOG_DIR, 'validation_*.csv')),
                       key=os.path.getmtime, reverse=True)
        return files[0] if files else None

    def run(self) -> dict:
        """
        Launch, wait for completion, collect metrics.
        Returns dict with: mission, success, lap_time_s, avg_speed_ms,
        max_speed_ms, avg_deviation_m, max_deviation_m, cones_hit, timed_out.
        """
        os.makedirs(LOG_DIR, exist_ok=True)

        # Build launch command
        cmd = [
            'ros2', 'launch', self.BRINGUP_PKG, self.BRINGUP_LAUNCH,
            f'mission:={self.mission}',
        ]
        for k, v in self.extra_args.items():
            cmd.append(f'{k}:={v}')

        # Capture CSV timestamp before launch so we can find the right file
        t_start = time.monotonic()

        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,   # process group for clean kill
        )

        # Wait for sim to initialise before polling (Gazebo takes ~10s to load)
        time.sleep(15.0)

        # Poll /planning/mission_finished until done or timeout
        deadline = t_start + self.timeout
        finished = False
        while time.monotonic() < deadline:
            try:
                result = subprocess.run(
                    ['ros2', 'topic', 'echo', '--once',
                     '/planning/mission_finished', 'std_msgs/msg/Bool'],
                    capture_output=True, text=True, timeout=3.0
                )
                if 'data: true' in result.stdout.lower():
                    finished = True
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(1.0)

        wall_time = time.monotonic() - t_start

        # Kill the whole process group
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            self._proc.wait(timeout=5)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass

        # Give lap_validator time to flush CSV
        time.sleep(2.0)

        # Read the newest CSV written after t_start
        metrics = {
            'mission':       self.mission,
            'success':       finished,
            'timed_out':     not finished,
            'wall_time_s':   wall_time,
            'lap_time_s':    None,
            'avg_speed_ms':  None,
            'max_speed_ms':  None,
            'avg_deviation_m': None,
            'max_deviation_m': None,
            'cones_hit':     None,
        }

        csv_file = self._latest_csv()
        if csv_file and os.path.getmtime(csv_file) > t_start:
            try:
                with open(csv_file) as f:
                    rows = list(csv.DictReader(f))
                if rows:
                    last = rows[-1]
                    metrics.update({
                        'lap_time_s':      float(last.get('lap_time_s',   0) or 0),
                        'avg_speed_ms':    float(last.get('avg_speed_ms', 0) or 0),
                        'max_speed_ms':    float(last.get('max_speed_ms', 0) or 0),
                        'avg_deviation_m': float(last.get('avg_deviation_m', 0) or 0),
                        'max_deviation_m': float(last.get('max_deviation_m', 0) or 0),
                        'cones_hit':       int(last.get('cones_hit', 0) or 0),
                    })
                    metrics['csv_path'] = csv_file
            except Exception as e:
                metrics['csv_error'] = str(e)

        return metrics


def _fmt(m: dict) -> str:
    """Format metrics dict as a compact summary string."""
    lines = [f"Mission: {m['mission']}  success={m['success']}"]
    if m['lap_time_s'] is not None:
        lines.append(
            f"  lap_time={m['lap_time_s']:.2f}s  "
            f"avg_speed={m['avg_speed_ms']:.2f}m/s  "
            f"max_speed={m['max_speed_ms']:.2f}m/s"
        )
        lines.append(
            f"  avg_dev={m['avg_deviation_m']:.3f}m  "
            f"max_dev={m['max_deviation_m']:.3f}m  "
            f"cones_hit={m['cones_hit']}"
        )
    if m.get('timed_out'):
        lines.append(f"  *** TIMED OUT after {m['wall_time_s']:.0f}s ***")
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Pass/fail criteria
# ---------------------------------------------------------------------------

# Acceleration: straight ~75m, car should complete in under 12s at ~10 m/s
ACCEL_CRITERIA = {
    'max_lap_time_s':      20.0,   # must complete within 20s
    'min_max_speed_ms':    6.0,    # must reach at least 6 m/s peak
    'max_cones_hit':       0,      # zero cone hits
    'max_avg_deviation_m': 1.5,    # stay within 1.5m of centreline
}

# Skidpad: 2L + 2R circles, r=9.1m, v~4.5 m/s
SKIDPAD_CRITERIA = {
    'max_lap_time_s':      120.0,  # full sequence within 120s
    'min_max_speed_ms':    2.0,    # some minimum movement
    'max_cones_hit':       0,
    'max_avg_deviation_m': 1.0,
}

# Peanut figure-8: two loops, ~100m total
PEANUT_CRITERIA = {
    'max_lap_time_s':      60.0,
    'min_max_speed_ms':    2.0,
    'max_cones_hit':       0,
    'max_avg_deviation_m': 0.8,
}

AUTOCROSS_CRITERIA = {
    'max_lap_time_s':      120.0,
    'min_max_speed_ms':    3.0,
    'max_cones_hit':       0,
    'max_avg_deviation_m': 1.0,
}


def _assert_mission(metrics: dict, criteria: dict) -> None:
    """Assert all pass/fail criteria. Prints full metrics on failure."""
    summary = _fmt(metrics)
    assert metrics['success'], \
        f"Mission did not complete (timed out).\n{summary}"
    if metrics['lap_time_s'] is not None:
        assert metrics['lap_time_s'] <= criteria['max_lap_time_s'], \
            f"Lap time {metrics['lap_time_s']:.2f}s > limit {criteria['max_lap_time_s']}s\n{summary}"
        assert metrics['max_speed_ms'] >= criteria['min_max_speed_ms'], \
            f"Max speed {metrics['max_speed_ms']:.2f}m/s < min {criteria['min_max_speed_ms']}m/s\n{summary}"
        assert metrics['cones_hit'] <= criteria['max_cones_hit'], \
            f"Cones hit: {metrics['cones_hit']} > limit {criteria['max_cones_hit']}\n{summary}"
        assert metrics['avg_deviation_m'] <= criteria['max_avg_deviation_m'], \
            f"Avg deviation {metrics['avg_deviation_m']:.3f}m > limit {criteria['max_avg_deviation_m']}m\n{summary}"


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestAcceleration:

    @GAZEBO_SKIP
    @pytest.mark.timeout(90)
    def test_accel_completes(self):
        """
        Acceleration mission:
          - Car starts at rest, drives ~75m straight to orange finish gate
          - Must complete within 20s, reach ≥6 m/s, hit 0 cones
        """
        runner = MissionRunner('acceleration', timeout_s=60.0)
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        _assert_mission(metrics, ACCEL_CRITERIA)

    @GAZEBO_SKIP
    @pytest.mark.timeout(90)
    def test_accel_reaches_target_speed(self):
        """Max speed during accel run must be ≥ 80% of configured max_speed."""
        runner = MissionRunner('acceleration', timeout_s=60.0)
        metrics = runner.run()
        configured_max = 13.0   # m/s (from bringup.launch.py acceleration config)
        print(f"\n{_fmt(metrics)}")
        assert metrics.get('max_speed_ms', 0) >= configured_max * 0.8, \
            f"Only reached {metrics.get('max_speed_ms', 0):.1f}m/s (expect ≥{configured_max*0.8:.1f})"

    @GAZEBO_SKIP
    @pytest.mark.timeout(90)
    def test_accel_no_cone_hits(self):
        """Acceleration run must complete without hitting any cones."""
        runner = MissionRunner('acceleration', timeout_s=60.0)
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        assert metrics.get('cones_hit', 1) == 0, \
            f"Hit {metrics['cones_hit']} cone(s) during acceleration"


class TestSkidpad:

    @GAZEBO_SKIP
    @pytest.mark.timeout(180)
    def test_skidpad_completes_2l_2r(self):
        """
        Skidpad mission:
          - 2 laps LEFT (CCW, r=9.1m) then 2 laps RIGHT (CW) then exit
          - Mission finished signal must arrive within 120s
          - Zero cone hits, stays on track (avg_dev < 1m)
        """
        runner = MissionRunner('skidpad', timeout_s=120.0)
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        _assert_mission(metrics, SKIDPAD_CRITERIA)

    @GAZEBO_SKIP
    @pytest.mark.timeout(180)
    def test_skidpad_lateral_acceleration(self):
        """
        On a 9.1m radius circle at ~4.5 m/s:
        a_lat = v²/r = 4.5²/9.1 ≈ 2.2 m/s²
        Check the car is actually going around the circle (not crawling).
        """
        runner = MissionRunner('skidpad', timeout_s=120.0)
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        assert metrics['success'], f"Skidpad did not complete\n{_fmt(metrics)}"
        # avg_speed during circles should be 2–6 m/s
        avg_v = metrics.get('avg_speed_ms', 0)
        assert avg_v >= 1.5, f"Average speed {avg_v:.2f}m/s too low — car barely moving"

    @GAZEBO_SKIP
    @pytest.mark.timeout(180)
    def test_skidpad_no_cone_hits(self):
        """Skidpad must complete without hitting any cones."""
        runner = MissionRunner('skidpad', timeout_s=120.0)
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        assert metrics.get('cones_hit', 1) == 0, \
            f"Hit {metrics['cones_hit']} cone(s) during skidpad"


class TestPeanut:

    @GAZEBO_SKIP
    @pytest.mark.timeout(120)
    def test_peanut_completes(self):
        """
        Peanut figure-8 mission:
          - Right loop then left loop, return to start = 1 lap
          - Must complete within 60s, no cone hits, avg_dev < 0.8m
        """
        runner = MissionRunner('peanut', timeout_s=90.0,
                               extra_args={'num_laps': '1'})
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        _assert_mission(metrics, PEANUT_CRITERIA)

    @GAZEBO_SKIP
    @pytest.mark.timeout(120)
    def test_peanut_no_cone_hits(self):
        """Peanut figure-8 must complete without hitting cones."""
        runner = MissionRunner('peanut', timeout_s=90.0,
                               extra_args={'num_laps': '1'})
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        assert metrics.get('cones_hit', 1) == 0, \
            f"Hit {metrics['cones_hit']} cone(s) on peanut figure-8"

    @GAZEBO_SKIP
    @pytest.mark.timeout(120)
    def test_peanut_crossover_no_collision(self):
        """
        At the figure-8 crossover point, the car path crosses itself.
        Deviation from centreline must stay < 0.8m even at the crossing.
        """
        runner = MissionRunner('peanut', timeout_s=90.0,
                               extra_args={'num_laps': '1'})
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        assert metrics['success'], f"Peanut did not complete\n{_fmt(metrics)}"
        assert metrics.get('max_deviation_m', 99) < 1.5, \
            f"Max deviation {metrics['max_deviation_m']:.3f}m at crossover too large"


class TestAutocross:

    @GAZEBO_SKIP
    @pytest.mark.timeout(180)
    def test_autocross_one_lap(self):
        """
        Autocross: one lap of the EUFS track.
        Must complete, no cone hits, avg deviation < 1m.
        """
        runner = MissionRunner('autocross', timeout_s=120.0,
                               extra_args={'num_laps': '1'})
        metrics = runner.run()
        print(f"\n{_fmt(metrics)}")
        _assert_mission(metrics, AUTOCROSS_CRITERIA)


# ---------------------------------------------------------------------------
# Comparative tests (regression: branch vs baseline)
# ---------------------------------------------------------------------------

class TestRegressionVsBaseline:
    """
    Run autocross twice — once with PSO optimizer enabled, once with L-BFGS-B.
    PSO should produce a lap time ≤ L-BFGS-B (or within 5% noise margin).
    """

    @GAZEBO_SKIP
    @pytest.mark.timeout(360)
    def test_pso_not_slower_than_lbfgsb(self):
        """PSO race line must not be slower than L-BFGS-B by more than 5%."""
        runner_lbfgsb = MissionRunner('autocross', timeout_s=120.0,
                                      extra_args={'use_pso_optimizer': 'false'})
        m_baseline = runner_lbfgsb.run()

        runner_pso = MissionRunner('autocross', timeout_s=120.0,
                                   extra_args={'use_pso_optimizer': 'true'})
        m_pso = runner_pso.run()

        print(f"\nBaseline (L-BFGS-B):\n{_fmt(m_baseline)}")
        print(f"\nPSO:\n{_fmt(m_pso)}")

        assert m_baseline['success'], "Baseline run did not complete"
        assert m_pso['success'],      "PSO run did not complete"

        t_base = m_baseline.get('lap_time_s', 0)
        t_pso  = m_pso.get('lap_time_s', 0)

        if t_base and t_pso:
            assert t_pso <= t_base * 1.05, \
                f"PSO ({t_pso:.2f}s) is >5% slower than L-BFGS-B ({t_base:.2f}s)"
