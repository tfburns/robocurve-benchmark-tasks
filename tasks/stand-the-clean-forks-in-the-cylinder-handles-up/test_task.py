"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import os
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena import demo as D  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402

import hhkit  # noqa: E402
import hhorient  # noqa: E402
import hhperson  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
SPEC = {o.name: o for o in M.OBJECTS}
PARKED = [{"tcp": np.array([0.12, 0.30, 0.95]), "tips": [np.array([0.12, 0.30, 0.93])] * 2}]


def quat_from_matrix(m):
    return ik.matrix_to_quat_xyzw(np.asarray(m, float))


def fork(name, centre, rotm=np.eye(3), vel=(0.0, 0.0, 0.0)):
    """The state the simulator reports for a fork whose collider centre is ``centre``."""
    c = np.asarray(centre, float) + np.asarray(rotm) @ hhperson.local_centre(SPEC[name])
    h = np.full(3, 0.1)
    return ObjectState(pos=c, quat_xyzw=quat_from_matrix(rotm), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - h, high=c + h)


def ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


HANDLES_UP = ry(80)  # local +x (tines) tipped down to 10 degrees off vertical, leaning on the wall
TINES_UP = ry(-80)
IN_Z = M.TOP + M.CYL_FLOOR + 0.095


def scene(**forks):
    st = {n: fork(n, (M.FORK_X, y, M.TOWEL_TOP + M.FORK[2] / 2)) for n, y in M.FORK_YS.items()}
    st.update(forks)
    return st


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = PARKED, np.zeros((0, 3)), None
    yield


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def test_loads_two_loose_forks_and_a_fixed_cylinder():
    assert set(TASK.free_objects) == {"fork_a", "fork_b"}
    assert all(o.mass is None for o in M.OBJECTS if o.name.startswith(("cylinder", "towel")))
    assert "up" not in M.INSTRUCTION.lower() and "tine" not in M.INSTRUCTION.lower()


def test_cylinder_is_a_closed_ring_of_real_flatware_cylinder_size():
    walls = [o for o in M.OBJECTS if o.name.startswith("cylinder_wall")]
    assert len(walls) == M.STAVES
    assert abs(2 * M.CYL_R_OUT - 0.095) < 0.002 and abs(M.CYL_H - 0.14) < 0.002  # Browne 80110: 3.75 x 5.5 in
    chord = 2 * (M.CYL_R_OUT - M.CYL_WALL / 2) * math.sin(math.pi / M.STAVES)
    assert walls[0].size[1] > chord


def test_start_scores_zero():
    assert step(scene()) == (False, 0.0)


def test_both_handles_up_scores_one():
    a = fork("fork_a", (M.CYL_XY[0] - 0.01, M.CYL_XY[1], IN_Z), HANDLES_UP)
    b = fork("fork_b", (M.CYL_XY[0] + 0.01, M.CYL_XY[1], IN_Z), HANDLES_UP)
    assert step(scene(fork_a=a, fork_b=b)) == (False, 1.0)  # still needs a second, unmoved step
    assert step(scene(fork_a=a, fork_b=b)) == (True, 1.0)


def test_one_handles_up_scores_half():
    a = fork("fork_a", (M.CYL_XY[0], M.CYL_XY[1], IN_Z), HANDLES_UP)
    assert step(scene(fork_a=a)) == (False, 0.5)


def test_any_fork_tines_up_scores_zero():
    a = fork("fork_a", (M.CYL_XY[0] - 0.01, M.CYL_XY[1], IN_Z), HANDLES_UP)
    b = fork("fork_b", (M.CYL_XY[0] + 0.01, M.CYL_XY[1], IN_Z), TINES_UP)
    assert step(scene(fork_a=a, fork_b=b)) == (False, 0.0)


def test_still_held_does_not_count():
    a = fork("fork_a", (M.CYL_XY[0], M.CYL_XY[1], IN_Z), HANDLES_UP)
    h, _t = M.ends(hhperson.body_states({"fork_a": a}, M.OBJECTS)["fork_a"])
    grip = [{"tcp": h, "tips": [h, h]}]
    assert step(scene(fork_a=a), grip)[1] == 0.0


def test_touching_the_tines_latches_zero():
    s = hhperson.body_states(scene(), M.OBJECTS)["fork_a"]
    _h, t = M.ends(s)
    assert step(scene(), [{"tcp": t + [0, 0, 0.004], "tips": [t, t]}])[1] == 0.0
    a = fork("fork_a", (M.CYL_XY[0] - 0.01, M.CYL_XY[1], IN_Z), HANDLES_UP)
    b = fork("fork_b", (M.CYL_XY[0] + 0.01, M.CYL_XY[1], IN_Z), HANDLES_UP)
    assert step(scene(fork_a=a, fork_b=b), PARKED) == (False, 0.0)


def test_holding_the_handle_end_is_not_touching_the_tines():
    s = hhperson.body_states(scene(), M.OBJECTS)["fork_a"]
    h, _t = M.ends(s)
    g = h + hhkit.rot(s.quat_xyzw)[:, 0] * M.GRIP_FROM_END
    step(scene(), [{"tcp": g, "tips": [g + [0, 0.01, -0.013], g + [0, -0.01, -0.013]]}])
    assert not M.TRIAL.mem["violated"]


def test_orient_wrap_keeps_plain_yaws_and_carries_tilts():
    assert np.allclose(D.grip_rotation(0.3), hhorient.plain(0.3))
    assert np.allclose(hhorient.approach(M.POINT), [1, 0, 0], atol=1e-9)
    assert np.allclose(D.grip_rotation(M.POINT), M.POINT.rot)


@pytest.mark.parametrize("variant", ["demo", "naive", "unsafe"])
def test_scripted_plans_track_within_3mm_and_2deg(variant, monkeypatch):
    monkeypatch.setenv("TASK_DEMO", variant)
    plan = D.ArmPlan(M.demo()["left"], D.LEFT_BASE_POS)
    t = 0.0
    while t < plan.duration:
        plan.action7(t)
        t += 1 / 20
    assert plan.worst_ik_mm < 3.0 and plan.worst_ik_deg < 2.0
