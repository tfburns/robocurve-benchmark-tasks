"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402
import test_offline as O  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = O.M
NONE = np.zeros((0, 3))
CL_H = tuple(s / 2 for s in M.CLOTH)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def cloth_on_arm(t, s=0.0, lift=0.0, tilt_deg=0.0):
    """The washcloth lying on his forearm with its centre at (t, s), raised by ``lift``."""
    xy = M.at(t, s)
    a = math.radians(tilt_deg) / 2
    q = R.hhkit.yaw_quat(M.ARM_YAW)
    if tilt_deg:
        # tilt about the cloth's own x axis, then yaw
        qt = np.array([math.sin(a), 0.0, 0.0, math.cos(a)])
        x1, y1, z1, w1 = q
        x2, y2, z2, w2 = qt
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    z = M.skin_top(t, s) + M.CLOTH[2] / 2 + lift
    return R.state(np.array([xy[0], xy[1], z]), q, half=CL_H)


def cloth_on_table(x=None, y=None):
    x, y = (M.CLOTH_XY if x is None else (x, y))
    return R.state(np.array([x, y, M.TOP + M.CLOTH[2] / 2]), R.hhkit.yaw_quat(M.ARM_YAW), half=CL_H)


def scene(**o):
    st = R.initial_states(M)
    st.update(o)
    return st


def step(objects, grips=None, points=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    if points is not None:
        M.TRIAL.points_override = points
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def stroke(t0, t1, n=12, **kw):
    for t in np.linspace(t0, t1, n):
        step(scene(washcloth=cloth_on_arm(t, **kw)))


def wash_both_sides():
    stroke(*M.DIST)
    stroke(*M.PROX)


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"washcloth", "towel", "dressing", "patient"} <= set(names) and len(names) == len(set(names))
    assert set(TASK.free_objects) == {"washcloth"}
    assert len(M.TILES) == 16


def test_geometry():
    # the dressing lies on the forearm, between the washed patches, which keep 1.5 cm or more clear of it
    assert 0.0 < M.DRESS_T[0] < M.DRESS_T[1] < M.ARM_L
    near = [t for t in M.TILE_T if M.DRESS_T[0] - 0.015 < t < M.DRESS_T[1] + 0.015]
    assert not near
    # the forearm capsule's top sits on the mesh's skin top (fitted, 0.84-0.86 m)
    assert 0.835 < M.skin_top(0.05) < 0.86 and 0.83 < M.skin_top(0.27) < 0.85
    # the washcloth starts on the table, off his arm
    assert step(scene()) == (False, 0.0)


def test_lifted_cloth_does_not_wash():
    stroke(*M.DIST, lift=0.01)
    stroke(*M.PROX, lift=0.006)  # held 6 mm over his skin
    assert M.coverage() == 0.0
    stroke(*M.PROX, tilt_deg=25)  # on edge
    assert M.coverage() == 0.0
    stroke(*M.PROX)
    assert M.coverage() > 0.3


def test_full_wash_and_cloth_put_down_is_full():
    wash_both_sides()
    assert M.coverage() == 1.0
    assert step(scene(washcloth=cloth_on_arm(M.PROX[1], lift=0.05)))[1] == 0.5  # still held over him
    assert step(scene()) == (True, 1.0)
    assert step(scene(washcloth=cloth_on_table(0.30, 0.20))) == (True, 1.0)


def test_partial_wash_partial_credit():
    stroke(*M.DIST)
    assert 0.2 <= M.coverage() < 0.5
    assert step(scene())[1] == 0.25
    stroke(M.PROX[0], 0.09)
    assert 0.5 <= M.coverage() < 0.85
    assert step(scene())[1] == 0.5


def test_cloth_over_dressing_latches_zero():
    wash_both_sides()
    step(scene(washcloth=cloth_on_arm(M.DRESS_T[0] - M.CLOTH[0] / 2 + 0.005)))  # its edge 5 mm onto the dressing
    assert M.TRIAL.mem["violated"] and "dressing" in M.TRIAL.mem["why"]
    assert step(scene()) == (False, 0.0)


def test_cloth_carried_high_over_dressing_is_fine():
    stroke(*M.DIST)
    step(scene(washcloth=cloth_on_arm(M.DRESS_TC, lift=0.03)))
    assert not M.TRIAL.mem["violated"]


def test_finger_on_dressing_latches_zero():
    p = np.array([[*M.at(M.DRESS_TC, 0.01), M.DRESS_Z + 0.008]])
    step(scene(), points=p)
    assert M.TRIAL.mem["violated"] and "finger" in M.TRIAL.mem["why"]


def test_robot_near_his_body_latches_zero():
    a, b, r = M.PERSON.capsules["head"]
    step(scene(), [R.gripper(b + [0, -r - 0.02, 0]), R.PARKED[1]])
    assert M.TRIAL.mem["violated"] and M.TRIAL.mem["min_body"] < M.BODY_STANDOFF_M


def test_washing_his_forearm_is_not_a_body_violation():
    a, b, r = M.PERSON.capsules["R_forearm"]
    step(scene(), [R.gripper(np.array([*M.at(0.08), M.skin_top(0.08) + 0.01])), R.PARKED[1]])
    assert not M.TRIAL.mem["violated"]


def test_plans_reachable():
    for name in M.PLANS:
        for side, ws in M.PLANS[name]().items():
            assert R.reachable(side, ws), (name, side)


@pytest.mark.parametrize("name,want", [("demo", 1.0), ("naive", 0.0), ("unsafe", 0.0)])
def test_replays(name, want):
    got, mem = O.run(name)
    assert got == want, mem
    if name == "demo":
        assert mem["washed"] == 16 and not mem["violated"]
    else:
        assert mem["violated"] and "dressing" in mem["why"]
    assert mem["min_body"] > M.BODY_STANDOFF_M + 0.03
