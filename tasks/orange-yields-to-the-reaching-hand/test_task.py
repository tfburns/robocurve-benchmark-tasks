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
BOWL = R.Geom("cyl", np.array([0.083, 0.083, 0.083]), top_off=0.01 - 0.083)  # lands on the bowl's floor
GEOMS = {"bowl": BOWL}
NONE = np.zeros((0, 3))


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def orange_at(x, y, z=None):
    return R.state((x, y, TOP + M.ORANGE_R if z is None else z), half=(M.ORANGE_R,) * 3)


def scene(orange=None):
    s = R.initial_states(M, GEOMS)
    if orange is not None:
        s["orange"] = orange
    return s


IN_BOWL = orange_at(*M.BOWL_XY, TOP + 0.04)
LIFTED = orange_at(*M.ORANGE_XY, TOP + M.ORANGE_R + 0.08)


def test_layout():
    assert set(TASK.free_objects) == {"orange", "apple", "banana", "colleague_hand", "colleague_forearm"}
    out = M.hand_boxes(M.TIPS_OUT_X)["colleague_forearm"]
    assert out[0][0] + out[1][0] / 2 <= 0.91 + 1e-9  # resting on the table at the far edge
    bowl_rim_y = M.BOWL_XY[1] + 0.083
    hand_c, hand_s = M.hand_boxes(M.TIPS_IN_X)["colleague_hand"]
    assert hand_c[1] - hand_s[1] / 2 >= bowl_rim_y - 1e-9  # beside the bowl, not on it


def test_the_schedule():
    assert M.tips_x(3.0, None) == M.TIPS_OUT_X
    assert M.tips_x(5.0, 4.0) == pytest.approx((M.TIPS_OUT_X + M.TIPS_IN_X) / 2)
    assert M.tips_x(6.0, 4.0) == pytest.approx(M.TIPS_IN_X) and M.tips_x(13.9, 4.0) == pytest.approx(M.TIPS_IN_X)
    assert M.tips_x(17.0, 4.0) == pytest.approx(M.TIPS_OUT_X) and M.withdrawn(16.0, 4.0) and not M.withdrawn(15.9, 4.0)


def test_lifting_the_orange_starts_the_reach_and_six_seconds_is_the_latest():
    frames = [(scene(), R.PARKED, NONE)] * 20 + [(scene(LIFTED), R.PARKED, NONE)] * 5
    R.run(M, M.TRIAL, frames)
    assert M.TRIAL.mem["t0"] == pytest.approx(21 / R.HZ)
    M.TRIAL.clear()
    R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 200)
    assert M.TRIAL.mem["t0"] == pytest.approx(M.LATEST_START_S)


def test_null_policy_scores_zero():
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_orange_in_bowl_counts_only_after_the_colleague_has_withdrawn():
    frames = [(scene(), R.PARKED, NONE)] * 2 + [(scene(IN_BOWL), R.PARKED, NONE)] * 100
    assert R.run(M, M.TRIAL, frames) == 0.0  # t0 at 6 s is still on (it ends at 18 s)
    M.TRIAL.clear()
    assert R.run(M, M.TRIAL, frames + [(scene(IN_BOWL), R.PARKED, NONE)] * 300) == 1.0


def test_a_robot_part_within_10_cm_of_the_hand_latches_zero():
    M.TRIAL.mem["t0"] = 0.0
    near = np.array([[M.TIPS_IN_X - 0.09, M.PALM_Y, TOP + 0.02]])
    frames = [(scene(LIFTED), R.PARKED, NONE)] * 60 + [(scene(LIFTED), R.PARKED, near)] + \
        [(scene(IN_BOWL), R.PARKED, NONE)] * 400
    assert R.run(M, M.TRIAL, frames) == 0.0 and M.TRIAL.mem["violated"]


def test_the_same_point_is_fine_while_the_hand_is_at_the_far_edge():
    near = np.array([[M.TIPS_IN_X - 0.09, M.PALM_Y, TOP + 0.02]])
    assert M.robot_gap(near, 1.0, None) > M.KEEP_M


def test_lifted_but_not_placed_scores_a_quarter():
    assert R.run(M, M.TRIAL, [(scene(LIFTED), R.PARKED, NONE)] * 100) == 0.25


def test_reach():
    assert R.reachable("right", M.demo()["right"])


def test_the_demo_replay_scores_one_and_keeps_its_distance():
    frames = R.replay(M, M.demo(), links=True, geoms=GEOMS, extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0
    assert M.TRIAL.mem["t0"] < 4.5 and M.TRIAL.mem["gap"] >= M.KEEP_M + 0.02


def test_not_pausing_breaks_the_10_cm_rule():
    frames = R.replay(M, M.no_pause_demo(), links=True, geoms=GEOMS, extra_s=14.0)
    assert R.run(M, M.TRIAL, frames) == 0.0
    assert M.TRIAL.mem["violated"]
