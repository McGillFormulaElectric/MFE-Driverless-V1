"""Race-line mission gate (core.raceline_runner) and its default (core.params). No ROS."""
import pytest

from mfe_path_planning.core.params import PlannerParams
from mfe_path_planning.core.raceline_runner import RACELINE_MISSIONS, RaceLineRunner, raceline_gate


def test_race_line_is_on_by_default():
    assert PlannerParams().raceline_enable is True


@pytest.mark.parametrize('mission', ['trackdrive', 'autocross', 'peanut'])
def test_gate_allows_closed_loop_missions(mission):
    assert raceline_gate(mission, True) is True


@pytest.mark.parametrize('mission', ['acceleration', 'skidpad', 'inspection', 'ebs', 'autonomous_demo',
                                     'some_track_name', ''])
def test_gate_blocks_every_other_mission(mission):
    assert raceline_gate(mission, True) is False


def test_gate_respects_the_enable_flag():
    assert raceline_gate('autocross', False) is False


def test_allowed_missions_are_exactly_the_closed_loop_ones():
    assert set(RACELINE_MISSIONS) == {'trackdrive', 'autocross', 'peanut'}


def test_disabled_runner_never_collects_or_publishes():
    runner = RaceLineRunner(raceline_gate('acceleration', True), 1, 0.25, 6.0, 8.0, 35.0)
    assert runner.state == 'off' and not runner.enabled
    assert runner.collect_wanted(0) is False
    window, events = runner.cycle((0.0, 0.0), 3, lambda: None)
    assert window is None and events == []
