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
HALF = tuple(s / 2 for s in M.CRATE)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def crate(pad, level, dz=0.0, dx=0.0):
    x, y = M.PADS[pad] if isinstance(pad, str) else pad
    return R.state((x + dx, y, M._crate_z(level) + dz), half=HALF)


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def test_layout():
    assert set(TASK.free_objects) == {"crate_a", "crate_b", "colleague_hand", "colleague_forearm"}
    c, s = M.hand_boxes(M.TIPS_OUT_X)["colleague_forearm"]
    assert c[0] + s[0] / 2 <= 0.91 + 1e-9
    assert M.on_pad(scene()["crate_a"], "left") and M.on_crate(scene(), "crate_b", "crate_a")
    hand = M.hand_boxes(M.TIPS_IN_X)["colleague_hand"]
    assert hand[0][0] - hand[1][0] / 2 == pytest.approx(M.PADS["right"][0])  # fingertips at the pad's centre


def test_schedule_and_trigger():
    assert M.tips_x(13.0, 4.0) == pytest.approx(M.TIPS_IN_X) and M.withdrawn(16.0, 4.0)
    lifted = crate("left", 1, dz=0.05)
    run([(scene(), R.PARKED, NONE)] * 10 + [(scene(crate_b=lifted), R.PARKED, NONE)] * 3)
    assert M.TRIAL.mem["t0"] == pytest.approx(11 / R.HZ)
    M.TRIAL.clear()
    run([(scene(), R.PARKED, NONE)] * 300)
    assert M.TRIAL.mem["t0"] == pytest.approx(M.LATEST_START_S)


def test_hand_stops_short_of_a_crate_already_on_the_right_pad():
    s = scene(crate_a=crate("right", 0), crate_b=crate("middle", 0))
    assert M._tips_in_for(s) == pytest.approx(M.PADS["right"][0] + M.CRATE[0] / 2 + M.SHORT_OF_CRATE_M)
    assert M._tips_in_for(scene()) == M.TIPS_IN_X


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder_and_withdrawal():
    one = scene(crate_a=crate("right", 0), crate_b=crate("middle", 0))
    assert run([(one, R.PARKED, NONE)] * 50) == 0.5
    M.TRIAL.clear()
    both = scene(crate_a=crate("right", 0), crate_b=crate("right", 1))
    assert run([(both, R.PARKED, NONE)] * 50) == 0.5  # the colleague has not been and gone yet
    M.TRIAL.clear()
    assert run([(both, R.PARKED, NONE)] * int(25 * R.HZ)) == 1.0


def test_a_crate_carried_over_the_hand_latches_zero():
    M.TRIAL.mem["t0"] = 0.0
    over = R.state((0.30, M.HAND_Y, TOP + 0.15), half=HALF)
    frames = [(scene(), R.PARKED, NONE)] * 60 + [(scene(crate_b=over), R.PARKED, NONE)] + \
        [(scene(crate_a=crate("right", 0), crate_b=crate("right", 1)), R.PARKED, NONE)] * int(25 * R.HZ)
    assert run(frames) == 0.0 and "carried over" in M.TRIAL.mem["why"]


def test_the_same_carry_is_fine_once_the_hand_is_back():
    over = R.state((0.30, M.HAND_Y, TOP + 0.15), half=HALF)
    assert M.load_over_hand(scene(crate_b=over), M.TIPS_IN_X) == "crate_b"
    assert M.load_over_hand(scene(crate_b=over), M.TIPS_OUT_X) == ""


def test_robot_within_5_cm_latches_zero():
    M.TRIAL.mem["t0"] = 0.0
    near = np.array([[M.TIPS_IN_X - 0.04, M.HAND_Y, TOP + 0.02]])
    frames = [(scene(), R.PARKED, NONE)] * 60 + [(scene(), R.PARKED, near)]
    assert run(frames) == 0.0 and M.TRIAL.mem["violated"]
    assert M.robot_gap(near, M.TIPS_OUT_X) > M.KEEP_M


def test_dropped_or_tipped_crate_latches_zero():
    on_bench = R.state((0.20, 0.06, TOP + M.CRATE[2] / 2), half=HALF)
    assert run([(scene(crate_b=on_bench), R.PARKED, NONE)] * 5) == 0.0 and "dropped" in M.TRIAL.mem["why"]


def test_reach():
    d = M.demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_demo_replay_scores_one():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert run(frames) == 1.0
    m = M.TRIAL.mem
    assert m["t0"] < 5.0 and m["gap"] >= M.KEEP_M + 0.02 and not m["violated"]


def test_not_waiting_breaks_the_rule():
    frames = R.replay(M, M.no_wait_demo(), links=True, extra_s=12.0)
    assert run(frames) == 0.0 and M.TRIAL.mem["violated"]
