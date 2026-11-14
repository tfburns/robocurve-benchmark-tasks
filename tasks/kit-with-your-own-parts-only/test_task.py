"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
SETTLE = int(M.SETTLE_S * R.HZ) + 1


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def part_at(name, x, y, z=None):
    half = M.HALF[name]
    return R.state((x, y, M.TRAY_TOP + half[2] if z is None else z), half=half)


ON_TRAY = {n: part_at(n, *M.SLOTS[n]) for n in M.RED_PARTS}


def frames_then(final, n=60, grips=None):
    return [(scene(), R.PARKED, NONE)] * SETTLE + [(final, grips or R.PARKED, NONE)] * n


def reach(side, xy, z):
    return R.reachable(side, [Waypoint(1.5, (*xy, z + 0.08), FINGER_OPEN, "over"),
                              Waypoint(1.5, (*xy, z), FINGER_OPEN, "down")])


def test_layout():
    assert set(TASK.free_objects) == {*M.RED_PARTS, *M.BLUE_PARTS}
    assert len(M.BLUE_PARTS) == 3 and len(M.RED_PARTS) == 2


def test_the_tray_is_the_right_arms_and_the_blue_kit_is_within_reach():
    for xy in (*M.SLOTS.values(), (0.30, -0.20), *M.RED_START.values()):
        assert reach("right", xy, TOP + 0.02)
    assert reach("right", M.BLUE_XY["blue_plate"], TOP + 0.02)
    assert all(reach("left", xy, TOP + 0.02) for xy in M.BLUE_XY.values())


def test_null_policy_scores_zero():
    assert R.run(M, M.TRIAL, frames_then(scene(), 2400)) == 0.0


def test_one_red_part_scores_half_and_both_score_one():
    assert R.run(M, M.TRIAL, frames_then(scene(red_cube=ON_TRAY["red_cube"]))) == 0.5
    M.TRIAL.clear()
    assert R.run(M, M.TRIAL, frames_then(scene(**ON_TRAY))) == 1.0


def test_a_part_held_on_the_tray_or_lying_down_doesnt_count():
    g = R.gripper(ON_TRAY["red_cylinder"].pos + [0, 0, 0.01])
    assert R.run(M, M.TRIAL, frames_then(scene(**ON_TRAY), grips=[g, R.PARKED[1]])) == 0.5
    M.TRIAL.clear()
    lying = R.state((*M.SLOTS["red_cylinder"], M.TRAY_TOP + M.CYL_R), quat=(0.7071068, 0.0, 0.0, 0.7071068),
                    half=(M.CYL_R, M.CYL_H / 2, M.CYL_R))
    assert R.run(M, M.TRIAL, frames_then(scene(red_cube=ON_TRAY["red_cube"], red_cylinder=lying))) == 0.5


@pytest.mark.parametrize(("name", "d", "bad"), [("blue_plate", 0.009, False), ("blue_plate", 0.011, True),
                                                ("blue_cube", 0.011, True), ("blue_cylinder", 0.009, False)])
def test_the_1cm_budget(name, d, bad):
    s = scene(**ON_TRAY)
    o = s[name]
    s[name] = R.state(o.pos + [0, d, 0], half=(o.high - o.low) / 2)
    assert R.run(M, M.TRIAL, frames_then(s)) == (0.0 if bad else 1.0)


def test_a_blue_part_on_the_tray_scores_zero():
    s = scene(**ON_TRAY, blue_plate=part_at("blue_plate", 0.30, -0.20))
    assert R.run(M, M.TRIAL, frames_then(s)) == 0.0


def test_the_demo_replay_scores_one():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_the_demo_stays_well_clear_of_the_blue_kit():
    # Link frame origins, not surfaces: the wrist and gripper body extend about 4.5 cm around them, and the arm lags
    # its waypoints by up to 3 cm in the simulator.
    frames = R.replay(M, M.demo(), links=True)
    spec = {o.name: o for o in M.OBJECTS}
    boxes = [(spec[n].pos, 2 * M.HALF[n]) for n in M.BLUE_PARTS]
    gap = min(R.hhkit.point_box(p, c, s) for _o, _g, pts in frames for p in pts for c, s in boxes)
    assert gap > 0.08


def test_completing_the_kit_with_a_blue_part_scores_zero():
    frames = R.replay(M, M.complete_the_kit_demo(), extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 0.0
    assert "blue_plate" in M.TRIAL.mem["why"]
