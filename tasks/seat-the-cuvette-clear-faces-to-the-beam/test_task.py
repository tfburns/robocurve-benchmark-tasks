"""Offline checks of task.py's scorer (no simulator): python -m pytest -q test_task.py

The cuvette going into the well is checked in Isaac Sim (see task.json "scores"); here the scorer is driven with
hand-made object states."""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
NONE = np.zeros((0, 3))


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def yawq(deg, tilt=0.0):
    a, t = math.radians(deg), math.radians(tilt)
    qz = np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])
    qx = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
    # q = qz * qx
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def cuv(xy, bottom, yaw_deg=0.0, tilt=0.0, vel=(0.0, 0.0, 0.0)):
    return R.state(np.array([xy[0], xy[1], bottom + M.CUV[2] / 2]), yawq(yaw_deg, tilt),
                   half=tuple(s / 2 for s in M.CUV), vel=vel)


def held(p, axis_deg, bottom):
    """A right gripper closed on the cuvette at p, its closing axis at axis_deg from world x; "tips" are the finger
    links' origins as the sim reports them (offset (0.081, 0.048) in the gripper frame, 77 mm above the TCP)."""
    tcp = np.array([p[0], p[1], bottom + M.CUV[2] - M.GRIP_BELOW_TOP])
    t = math.radians(axis_deg)
    c, s = math.cos(t), math.sin(t)
    a, b = 0.081, M.FINGER_ORIGIN_SKEW
    d = np.array([a * c - b * s, a * s + b * c, 0.0])
    up = np.array([0.0, 0.0, 0.077])
    return [R.PARKED[0], {"tcp": tcp, "tips": [tcp + up + d / 2, tcp + up - d / 2]}]


def test_closing_axis_matches_sim_readings():
    # finger-link origins logged in Isaac Sim (v10, 17 Nov): yaw-0 grip closed on the cuvette, then the yaw-90 grip
    g0 = {"tcp": [0.4, -0.241, 0.782], "tips": [[0.2737 + 0.1263, -0.249 + 0.0, 0.86], [0.2255 + 0.1263, -0.232 - 0.0, 0.86]]}
    shut, ax = M.closing(g0)
    assert not shut  # 17 mm offset: still open
    g1 = {"tcp": [0.4, -0.24, 0.78], "tips": [[0.4243, -0.2721, 0.86], [0.3757, -0.2079, 0.86]]}
    shut, ax = M.closing(g1)
    assert shut and abs(abs(ax[1]) - 1.0) < 0.01  # closing along world y


def step(objects, grips=None):
    M.TRIAL.grippers_override = grips if grips is not None else R.PARKED
    M.TRIAL.new_step()
    return M.success(objects), M.score(objects)


def base():
    return R.initial_states(M)


WELL = M._well_xy()
RACK = M.RACK_XY
FLOOR = M.TOP + M.FLOOR_H
LIFT = M.TOP + 0.12


def test_sizes_are_a_standard_cuvette_and_its_well():
    assert M.CUV[:2] == (0.0125, 0.0125) and M.CUV[2] == 0.045
    assert M.SLOT - M.CUV[0] == pytest.approx(0.0035)
    assert M.CUV[2] - M.RING_H + M.RELEASE_ABOVE > M.GRIP_BELOW_TOP + 0.013 + 0.004  # fingertips clear of the instrument top


def test_well_is_where_the_plan_puts_it():
    st = base()
    w = M.well(st)
    assert np.allclose(w[:2], WELL, atol=1e-6) and w[2] == pytest.approx(FLOOR)


def test_start_scores_zero_and_is_in_the_rack():
    st = base()
    assert M.cuv_bottom(st) == pytest.approx(M.TOP + M.RACK_PLATE[2])
    assert step(st) == (False, 0.0)


def test_lifted_by_the_frosted_sides_is_quarter():
    st = base()
    st["cuvette"] = cuv(RACK, LIFT, 0.0)
    assert step(st, held(RACK, 90.0, LIFT)) == (False, 0.25)  # closing along y: pads on the +-y frosted sides


def test_held_by_the_clear_windows_latches_zero():
    st = base()
    st["cuvette"] = cuv(RACK, LIFT, 0.0)
    assert step(st, held(RACK, 0.0, LIFT)) == (False, 0.0)
    assert "windows" in M.TRIAL.mem["why"]
    st["cuvette"] = cuv(WELL, FLOOR, 90.0)
    assert step(st) == (False, 0.0)


def test_seated_turned_and_let_go_is_full():
    st = base()
    st["cuvette"] = cuv(WELL, FLOOR, 90.0)
    assert M.orient_err_deg(st) == pytest.approx(0.0, abs=1e-3)
    assert step(st) == (True, 1.0)
    st["cuvette"] = cuv(WELL, FLOOR, -90.0)
    assert step(st) == (True, 1.0)


def test_seated_but_still_held_is_half():
    st = base()
    st["cuvette"] = cuv(WELL, FLOOR, 90.0)
    assert step(st, held(WELL, 0.0, FLOOR)) == (False, 0.5)  # cuvette turned 90: world-x closing is on its frosted y


def test_seated_the_wrong_way_round_is_zero():
    st = base()
    st["cuvette"] = cuv(WELL, FLOOR, 0.0)
    assert step(st) == (False, 0.0)
    st["cuvette"] = cuv(WELL, FLOOR, 90.0 - 15.0)
    assert step(st) == (False, 0.0)


def test_nearly_turned_counts():
    st = base()
    st["cuvette"] = cuv(WELL, FLOOR, 90.0 - 8.0)
    assert step(st) == (True, 1.0)


def test_tipping_spills_and_latches():
    st = base()
    st["cuvette"] = cuv(RACK, LIFT, 0.0, tilt=30.0)
    assert step(st, held(RACK, 90.0, LIFT)) == (False, 0.0)
    assert "tipped" in M.TRIAL.mem["why"]
    st["cuvette"] = cuv(WELL, FLOOR, 90.0)
    assert step(st) == (False, 0.0)


def test_dropping_latches():
    st = base()
    st["cuvette"] = cuv(WELL, FLOOR + 0.02, 90.0, vel=(0.0, 0.0, -0.8))
    assert step(st) == (False, 0.0)
    assert "dropped" in M.TRIAL.mem["why"]


def test_plans_cover_the_variants_and_turn_the_right_way():
    import os
    for v, (g, p) in {"demo": (0.0, math.pi / 2), "naive": (0.0, 0.0), "unsafe": (math.pi / 2, 0.0)}.items():
        os.environ["TASK_DEMO"] = v
        plan = M.demo()["right"]
        assert plan[2].yaw == pytest.approx(g) and plan[10].yaw == pytest.approx(p + (M.TURN_TRIM if p != g else 0.0))
    os.environ.pop("TASK_DEMO")


def test_task_file_loads():
    assert TASK.module.INSTRUCTION == "Put the sample in the spectrophotometer."
    assert {o.name for o in M.OBJECTS} == {"cuvette", "rack", "spectrophotometer"}
