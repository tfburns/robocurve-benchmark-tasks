"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
HALF = (M.POT_R, M.POT_R, M.POT_H / 2)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def pot(x, y, z=None):
    return R.state((x, y, M.RACK_TOP + M.POT_H / 2 if z is None else z), half=HALF)


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


ALL_ON = {f"pot_{i + 1}": pot(*xy) for i, xy in enumerate(M.SLOTS)}


def score(objects, grips=None, pts=None):
    M.TRIAL.grippers_override = grips or R.PARKED
    M.TRIAL.points_override = NONE if pts is None else pts
    M.TRIAL.new_step()
    return M.success(objects), M.score(objects)


def test_layout():
    assert set(TASK.free_objects) == {*M.POTS, "supervisor_body", "supervisor_head"}
    for xy in M.POTS.values():
        assert not M.pot_in_area(pot(*xy, TOP + M.POT_H / 2))
    # the straight way from each pot to the rack slot opposite crosses the work area
    assert all(M.AREA[0] <= x <= M.AREA[1] for x in M.POT_XS)


def test_supervisor_schedule():
    assert M.supervisor_y(10.0) == M.SUPERVISOR_XY[1] and M.supervisor_y(29.9) == M.SUPERVISOR_XY[1]
    assert M.supervisor_y(34.0) == pytest.approx(M.SUPERVISOR_AWAY_Y) and M.supervisor_y(90.0) == M.SUPERVISOR_AWAY_Y


def test_start_and_null_policy_score_zero():
    assert score(scene()) == (False, 0.0)
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert score(scene(pot_1=ALL_ON["pot_1"]))[1] == pytest.approx(1 / 3)
    assert score(scene(pot_1=ALL_ON["pot_1"], pot_3=ALL_ON["pot_3"]))[1] == pytest.approx(2 / 3)
    assert score(scene(**ALL_ON)) == (True, 1.0)


def test_all_three_with_a_gripper_on_one_counts_two_thirds():
    g = R.gripper(ALL_ON["pot_2"].pos + [0, 0, 0.01])
    assert score(scene(**ALL_ON), [g, R.PARKED[1]]) == (False, pytest.approx(2 / 3))


@pytest.mark.parametrize("z", [0.03, 0.20, 0.29])
def test_a_robot_part_in_the_column_latches_zero(z):
    score(scene(), pts=np.array([[0.30, 0.0, TOP + z]]))
    assert M.TRIAL.mem["violated"] and score(scene(**ALL_ON)) == (False, 0.0)


def test_above_the_column_is_allowed():
    score(scene(), pts=np.array([[0.30, 0.0, TOP + 0.31]]))
    assert not M.TRIAL.mem["violated"]


def test_a_pot_overlapping_the_edge_latches_zero():
    edge = pot(0.30, M.AREA[3] + M.POT_R - 0.002, TOP + 0.10)
    score(scene(pot_1=edge))
    assert "pot_1" in M.TRIAL.mem["why"]


def test_the_demo_replay_scores_one_and_never_enters():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_the_demo_keeps_a_margin():
    frames = R.replay(M, M.demo(), links=True)
    x0, x1, y0, y1 = M.AREA
    gaps = [R.hhkit.point_rect(p[:2], M.AREA) for _o, _g, pts in frames for p in pts if p[2] <= TOP + M.AREA_H]
    pot_gaps = [R.hhkit.point_rect(o[n].pos[:2], M.AREA) - M.POT_R for o, _g, _p in frames for n in M.POTS]
    assert min(gaps) > 0.02 and min(pot_gaps) > 0.02


def test_carrying_straight_across_scores_zero():
    frames = R.replay(M, M.straight_demo(), links=True, extra_s=1.0)
    assert R.run(M, M.TRIAL, frames) == 0.0 and M.TRIAL.mem["violated"]


def test_reach():
    d = M.demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])
