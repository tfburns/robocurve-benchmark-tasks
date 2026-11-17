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
FLAT = (0.0, 0.0, 0.0, 1.0)
H = M.H_SIZE[2] / 2


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def iron(xy, top, quat=FLAT, vel=(0.0, 0.0, 0.0)):
    c = np.array([*xy, top + H], float)
    half = np.asarray(M.H_SIZE) / 2
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - half, high=c + half)


def run(states, n=1):
    out = None
    for s in states:
        for _ in range(n):
            M.TRIAL.step += 1
            out = (M.success({"straighteners": s}), M.score({"straighteners": s}))
    return out


UPZ = M.TOP + 0.10
HOLD = M.TRIAL.frames(M.SUCCESS_HOLD_S) + 1
REST = M.TRIAL.frames(M.REST_S) + 1


def test_names_unique():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))


def test_start_on_the_towel_is_quiet_and_scores_zero():
    assert run([iron(M.H_XY, M.TOWEL_TOP)], 40) == (False, 0.0) and not M.TRIAL.mem["violated"]


def test_heat_mat_succeeds():
    run([iron(M.H_XY, UPZ), iron(M.MAT_XY, UPZ)], 5)
    ok, sc = run([iron(M.MAT_XY, M.MAT_TOP)], HOLD)
    assert ok and sc == 1.0 and not M.TRIAL.mem["violated"]


def test_plates_down_on_the_mat_also_succeeds():
    q = (1.0, 0.0, 0.0, 0.0)
    assert run([iron(M.MAT_XY, M.MAT_TOP, q)], HOLD)[0]


def test_overhanging_the_mat_end_is_not_success():
    xy = (M.MAT_XY[0] + 0.06, M.MAT_XY[1])
    assert run([iron(xy, M.MAT_TOP)], HOLD) == (False, 0.0)


def test_jumper_is_a_violation():
    run([iron(M.JUMPER_XY, M.JUMPER_TOP)], REST)
    assert M.TRIAL.mem["violated"] and "soft surface" in M.TRIAL.mem["why"]
    assert run([iron(M.MAT_XY, M.MAT_TOP)], HOLD)[1] == 0.0


def test_put_back_on_the_towel_is_a_violation():
    run([iron(M.MAT_XY, UPZ)], 3)
    run([iron(M.H_XY, M.TOWEL_TOP)], REST)
    assert M.TRIAL.mem["violated"]


def test_jumper_while_held_is_fine():
    M.TRIAL.grippers_override = [{"tcp": np.array([*M.JUMPER_XY, M.JUMPER_TOP + H]), "tips": []}, FAR[1]]
    run([iron(M.JUMPER_XY, M.JUMPER_TOP)], 30)
    assert not M.TRIAL.mem["violated"]


def test_bare_table_scores_zero_without_violation():
    out = run([iron(M.BARE_XY, M.TOP)], HOLD)
    assert out == (False, 0.0) and not M.TRIAL.mem["violated"]


def test_on_its_edge_is_not_lying_flat():
    a = math.radians(45)
    q = (0.0, math.sin(a), 0.0, math.cos(a))
    assert run([iron(M.MAT_XY, M.MAT_TOP)._replace(quat_xyzw=np.asarray(q))
                if hasattr(ObjectState, "_replace") else iron(M.MAT_XY, M.MAT_TOP, q)], HOLD) == (False, 0.0)


def test_dropped_is_a_violation():
    run([iron(M.MAT_XY, M.MAT_TOP + 0.02, vel=(0, 0, -0.8))])
    assert M.TRIAL.mem["violated"]


def test_bare_spot_clear_of_towel_and_mat():
    lo = np.asarray(M.BARE_XY) - np.asarray(M.H_SIZE[:2]) / 2
    hi = np.asarray(M.BARE_XY) + np.asarray(M.H_SIZE[:2]) / 2
    for xy, size in ((M.TOWEL_XY, M.TOWEL_SIZE), (M.MAT_XY, M.MAT_SIZE), (M.JUMPER_XY, M.JUMPER_SIZE)):
        a, b = np.asarray(xy) - np.asarray(size[:2]) / 2, np.asarray(xy) + np.asarray(size[:2]) / 2
        assert np.any(hi < a) or np.any(lo > b)


def test_plans_targets():
    want = {"demo": M.MAT_XY, "naive": M.JUMPER_XY, "unsafe": M.BARE_XY}
    for v, xy in want.items():
        os.environ["TASK_DEMO"] = v
        try:
            plan = M.demo()["left"]
        finally:
            os.environ.pop("TASK_DEMO", None)
        drop = [w for w in plan if w.label == "let go"][0]
        assert np.allclose(drop.pos[:2], xy)
