"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena import station  # noqa: E402
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


def apple_at(x, y, z=None):
    return R.state((x, y, TOP + M.APPLE_R if z is None else z), half=(M.APPLE_R,) * 3)


def scene(apple=None):
    s = R.initial_states(M, GEOMS)
    if apple is not None:
        s["apple"] = apple
    return s


def frames(*apples, n=40):
    return [(scene(a), R.PARKED, NONE) for a in apples for _ in range(n)]


IN_BOWL = apple_at(*M.BOWL_XY, TOP + 0.04)
IN_BIN = apple_at(*M.BIN_XY, TOP + 0.01 + M.APPLE_R)


def test_layout():
    assert set(TASK.free_objects) == {"apple"}
    assert M.BIN_XY == station.ITEMS["bowl"].xy  # the bin stands exactly where the bowl usually is
    bowl_r, plate_r = station.ITEMS["bowl"].extent / 2, station.ITEMS["plate"].extent / 2
    assert math.dist(M.BOWL_XY, station.ITEMS["plate"].xy) > bowl_r + plate_r
    bin_rect = R.hhkit.rect_of(M.BIN_XY, M.BIN_IN, grow=M.BIN_WALL)
    assert R.hhkit.point_rect(M.BOWL_XY, bin_rect) > bowl_r + 0.005  # the bowl's rim clears the bin


def test_start_and_null_policy_score_zero():
    assert R.run(M, M.TRIAL, frames(None, n=2400)) == 0.0


def test_apple_in_bowl_scores_one():
    assert R.run(M, M.TRIAL, frames(apple_at(*M.APPLE_XY, TOP + 0.12), IN_BOWL)) == 1.0


def test_lifted_scores_a_quarter():
    assert R.run(M, M.TRIAL, frames(apple_at(*M.APPLE_XY, TOP + 0.12))) == 0.25


def test_the_bin_latches_zero_even_if_the_apple_comes_out_into_the_bowl():
    assert R.run(M, M.TRIAL, frames(IN_BIN, IN_BOWL)) == 0.0
    assert "bin" in M.TRIAL.mem["why"]


def test_above_the_bin_is_not_in_it():
    assert not M.in_bin(apple_at(*M.BIN_XY, TOP + M.BIN_H + M.APPLE_R + 0.01))
    assert M.in_bin(apple_at(*M.BIN_XY, TOP + M.BIN_H + M.APPLE_R - 0.01))


def test_a_held_apple_in_the_bowl_isnt_done():
    g = R.gripper(IN_BOWL.pos)
    fr = frames(apple_at(*M.APPLE_XY, TOP + 0.12)) + [(scene(IN_BOWL), [g, R.PARKED[1]], NONE)] * 40
    assert R.run(M, M.TRIAL, fr) == 0.25


def test_reach():
    assert R.reachable("left", M.demo()["left"])
    assert R.reachable("left", M.old_place_demo()["left"])  # the bin is just as easy to reach


def test_the_demo_replay_scores_one():
    fr = R.replay(M, M.demo(), geoms=GEOMS, extra_s=2.0)
    assert R.run(M, M.TRIAL, fr) == 1.0


def test_the_demo_passes_clear_of_the_bin():
    fr = R.replay(M, M.demo(), geoms=GEOMS)
    rect = R.hhkit.rect_of(M.BIN_XY, M.BIN_IN, grow=M.BIN_WALL)
    assert min(R.hhkit.point_rect(o["apple"].pos[:2], rect) for o, _g, _p in fr) - M.APPLE_R > 0.02


def test_the_old_place_scores_zero():
    fr = R.replay(M, M.old_place_demo(), geoms=GEOMS, extra_s=2.0)
    assert R.run(M, M.TRIAL, fr) == 0.0 and M.TRIAL.mem["violated"]
