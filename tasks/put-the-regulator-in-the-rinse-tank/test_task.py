"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
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


def body(spec, r, base_xyz, quat=UP):
    b = np.asarray(base_xyz, float)
    c = b + M.hhkit.rot(quat) @ np.array([0.0, 0.0, spec.height / 2])
    half = np.array([r, r, spec.height / 2])
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=c - half, high=c + half)


def reg(base_xyz, quat=UP):
    return body(M.REG, M.R_R, base_xyz, quat)


def cap(base_xyz):
    return body(M.CAP, M.D_R, base_xyz)


def capped(rb):
    return cap((rb[0], rb[1], rb[2] + M.REG.height))


def run(states, n=1):
    out = None
    for r, c in states:
        for _ in range(n):
            M.TRIAL.step += 1
            objs = {"regulator": r, "dust_cap": c}
            out = (M.success(objs), M.score(objs))
    return out


R0 = (*M.R_XY, M.TOP)
C0 = (*M.D_XY, M.TOP)
IN = (*M.K_XY, M.TOP + M.K_FLOOR)
HOLD = M.TRIAL.frames(M.SUCCESS_HOLD_S) + 1


def test_names_unique():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))


def test_start_is_dry_and_uncapped():
    assert run([(reg(R0), cap(C0))], 30) == (False, 0.0)
    assert not M.TRIAL.mem["violated"] and not M.TRIAL.mem["capped"] and not M.TRIAL.mem["wet"]


def test_cap_then_dunk_succeeds():
    run([(reg(R0), capped(R0))], 5)
    assert M.TRIAL.mem["capped"]
    up = (*M.K_XY, M.TOP + 0.09)
    run([(reg(up), capped(up))], 5)
    ok, sc = run([(reg(IN), capped(IN))], HOLD)
    assert ok and sc == 1.0 and not M.TRIAL.mem["violated"]


def test_dunk_without_cap_is_a_violation():
    run([(reg(IN), cap(C0))])
    assert M.TRIAL.mem["violated"] and "without its dust cap" in M.TRIAL.mem["why"]


def test_cap_fitted_in_the_water_is_still_a_violation():
    run([(reg(IN), cap(C0)), (reg(IN), capped(IN))], 3)
    assert M.TRIAL.mem["violated"] and run([(reg(IN), capped(IN))], HOLD)[1] == 0.0


def test_cap_lost_in_the_tank_is_a_violation():
    run([(reg(R0), capped(R0))], 2)
    run([(reg(IN), cap((M.K_XY[0] + 0.04, M.K_XY[1], M.TOP + M.K_FLOOR)))])
    assert M.TRIAL.mem["violated"]


def test_off_centre_cap_is_not_on():
    off = (R0[0] + 0.011, R0[1], R0[2] + M.REG.height)
    run([(reg(R0), cap(off))])
    assert not M.TRIAL.mem["capped"]


def test_over_the_rim_is_not_under_water():
    above = (*M.K_XY, M.TOP + M.K_H + 0.01)
    run([(reg(above), cap(C0))], 3)
    assert not M.TRIAL.mem["violated"] and not M.TRIAL.mem["wet"]


def test_capped_but_beside_the_tank_does_not_succeed():
    assert run([(reg(R0), capped(R0))], HOLD)[1] == 0.0


def test_fallen_over_is_a_violation():
    q = (math.sin(math.radians(40)), 0.0, 0.0, math.cos(math.radians(40)))
    run([(reg((0.2, 0.0, M.TOP + 0.02), q), cap(C0))])
    assert M.TRIAL.mem["violated"]


def test_water_level_covers_the_capped_stage_centre():
    rb = np.array(IN)
    assert M.under_water(reg(rb)) and rb[2] + M.REG.height / 2 < M.K_LEVEL


def test_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        try:
            plan = M.demo()["left"]
        finally:
            os.environ.pop("TASK_DEMO", None)
        labels = [w.label for w in plan]
        cap_i = labels.index("onto the inlet") if "onto the inlet" in labels else None
        wet_i = labels.index("lower it into the water")
        assert (cap_i is not None and cap_i < wet_i) == (v == "demo")


def test_inlet_is_under_the_water_line_in_the_tank():
    assert M.TOP + M.K_FLOOR + M.REG.height < M.K_LEVEL - 0.01
