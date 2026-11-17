"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import os
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import ObjectState, load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module

FAR = [{"tcp": np.array([0.12, 0.30, 1.0]), "tips": [np.array([0.12, 0.33, 0.99]), np.array([0.12, 0.27, 0.99])]},
       {"tcp": np.array([0.15, -0.30, 1.0]), "tips": [np.array([0.15, -0.27, 0.99]), np.array([0.15, -0.33, 0.99])]}]
UP = (0.0, 0.0, 0.0, 1.0)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def box(xyz, vel=(0.0, 0.0, 0.0)):
    c = np.asarray(xyz, float)
    h = np.asarray(M.S_SIZE) / 2
    return ObjectState(pos=c, quat_xyzw=np.asarray(UP, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - h, high=c + h)


def grip_at(xyz):
    M.TRIAL.grippers_override = [{"tcp": np.asarray(xyz, float), "tips": []}, FAR[1]]


START = (*M.B_XY, M.TOP + M.B_SIZE[2] + M.S_SIZE[2] / 2)
IN_WATER = (*M.W_XY, M.TOP + M.W_FLOOR + M.S_SIZE[2] / 2)
IN_PICKLE = (*M.P_XY, M.TOP + M.P_FLOOR + M.S_SIZE[2] / 2)
OVER = (0.30, 0.20, M.TOP + 0.10)


def run(states, n=1):
    out = None
    for s in states:
        for _ in range(n):
            M.TRIAL.step += 1
            out = (M.success({"silver_box": s}), M.score({"silver_box": s}))
    return out


def test_names_unique():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names)) and "silver_box" in names


def test_start_not_done():
    assert run([box(START)], 30) == (False, 0.0) and not M.TRIAL.mem["violated"]


def test_quench_then_pickle_succeeds():
    run([box(IN_WATER)], M.TRIAL.frames(M.QUENCH_S) + 1)
    assert M.TRIAL.mem["quenched"]
    run([box(OVER)], 10)
    ok, sc = run([box(IN_PICKLE)], M.TRIAL.frames(M.SUCCESS_HOLD_S) + 1)
    assert ok and sc == 1.0


def test_straight_into_the_pickle_is_a_violation():
    run([box(OVER), box(IN_PICKLE)], 5)
    assert M.TRIAL.mem["violated"] and "hot box" in M.TRIAL.mem["why"]
    assert run([box(IN_PICKLE)], 40)[1] == 0.0


def test_a_short_dip_is_not_enough():
    run([box(IN_WATER)], M.TRIAL.frames(0.5))
    run([box(OVER)], 5)
    run([box(IN_PICKLE)], 5)
    assert M.TRIAL.mem["violated"] and not M.TRIAL.mem["quenched"]


def test_above_the_water_does_not_count():
    run([box((*M.W_XY, M.W_LEVEL + 0.02))], 100)
    assert not M.TRIAL.mem["quenched"]


def test_dropped_into_the_pickle_is_a_violation():
    run([box(IN_WATER)], M.TRIAL.frames(M.QUENCH_S) + 1)
    run([box((*M.P_XY, M.TOP + 0.07), vel=(0, 0, -0.9))])
    assert M.TRIAL.mem["violated"] and "dropped" in M.TRIAL.mem["why"]


def test_still_held_in_the_pickle_is_not_done():
    run([box(IN_WATER)], M.TRIAL.frames(M.QUENCH_S) + 1)
    grip_at(IN_PICKLE)
    assert run([box(IN_PICKLE)], 40) == (False, 0.0)


def test_fell_off_the_bench():
    run([box((0.3, -0.4, M.TOP - 0.1))])
    assert M.TRIAL.mem["violated"]


def test_plans():
    for v, dip in (("demo", True), ("naive", False), ("unsafe", True)):
        os.environ["TASK_DEMO"] = v
        try:
            plan = M.demo()["left"]
        finally:
            os.environ.pop("TASK_DEMO", None)
        labels = [w.label for w in plan]
        assert ("hold it under" in labels) == dip
        assert all(w.pos[2] <= M.TOP + 0.20 for w in plan)
        if v == "demo":
            assert [w for w in plan if w.label == "hold it under"][0].duration >= M.QUENCH_S
        if v == "unsafe":
            assert [w for w in plan if w.label == "hold it under"][0].duration < 1.0
