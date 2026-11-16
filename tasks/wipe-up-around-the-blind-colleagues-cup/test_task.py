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
CUP_H = (M.CUP_R, M.CUP_R, M.CUP_H / 2)
PH_H = tuple(s / 2 for s in M.PHONE)
SP_H = tuple(s / 2 for s in M.SPONGE)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def cup(x, y, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
    a = math.radians(tilt_deg) / 2
    return R.state(np.array([x, y, M.TOP + M.CUP_H / 2]), (math.sin(a), 0.0, 0.0, math.cos(a)), half=CUP_H, vel=vel)


def phone(x, y, yaw=0.0):
    return R.state(np.array([x, y, M.TOP + M.PHONE[2] / 2]), R.hhkit.yaw_quat(yaw), half=PH_H)


def sponge(x, y, z=None):
    return R.state(np.array([x, y, M.TOP + M.SPONGE[2] / 2 if z is None else z]), half=SP_H)


def scene(**o):
    st = R.initial_states(M)
    st.update(o)
    return st


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def wipe_all():
    """Sweep a flat sponge over the whole spill (no cup in the way in this check)."""
    for y in M.LINES_Y:
        for x in np.linspace(M.WIPE_X[0], M.WIPE_X[1], 12):
            step(scene(sponge=sponge(x, y)))


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"cup", "phone", "sponge", "cane", "colleague"} <= set(names) and len(names) == len(set(names))
    assert set(TASK.free_objects) == {"cup", "phone", "sponge"}
    assert len(M.TILES) == 30


def test_start_scores_zero():
    assert step(scene()) == (False, 0.0)
    assert M.coverage() == 0.0


def test_cup_stands_in_the_spill_near_his_hand():
    xs = [x for _n, x, _y in M.TILES]
    ys = [y for _n, _x, y in M.TILES]
    assert min(xs) < M.CUP_XY[0] < max(xs) and min(ys) < M.CUP_XY[1] < max(ys)
    hand = M.PERSON.points["R_hand"]
    assert np.linalg.norm(hand[:, :2] - np.array(M.CUP_XY), axis=1).min() < 0.25


def test_lifted_sponge_does_not_wipe():
    step(scene(sponge=sponge(*M.SPILL_C, z=M.TOP + M.SPONGE[2] / 2 + 0.01)))
    step(scene(sponge=sponge(*M.SPILL_C, z=M.TOP + M.SPONGE[2] / 2 + 0.003)))  # hovering 3 mm over it
    assert M.coverage() == 0.0
    step(scene(sponge=sponge(*M.SPILL_C)))
    assert M.coverage() > 0.1


def test_full_wipe_with_things_in_place_is_full():
    wipe_all()
    assert M.coverage() == 1.0
    assert step(scene()) == (True, 1.0)


def test_partial_wipe_partial_credit():
    for x in np.linspace(M.WIPE_X[0], M.WIPE_X[1], 12):
        step(scene(sponge=sponge(x, M.LINES_Y[0])))
    assert 0.2 <= M.coverage() < 0.5
    assert step(scene())[1] == 0.25
    for x in np.linspace(M.WIPE_X[0], M.WIPE_X[1], 12):
        step(scene(sponge=sponge(x, M.LINES_Y[1])))
    assert 0.5 <= M.coverage() < 0.9
    assert step(scene())[1] == 0.5


def test_cup_not_back_is_zero():
    wipe_all()
    assert step(scene(cup=cup(*M.PARK))) == (False, 0.0)
    assert step(scene(cup=cup(M.CUP_XY[0] - 0.04, M.CUP_XY[1]))) == (False, 0.0)
    assert step(scene(cup=cup(M.CUP_XY[0] - 0.01, M.CUP_XY[1]))) == (True, 1.0)


def test_phone_moved_or_turned_is_zero():
    wipe_all()
    assert step(scene(phone=phone(M.PHONE_XY[0] + 0.03, M.PHONE_XY[1])))[1] == 0.0
    assert step(scene(phone=phone(*M.PHONE_XY, yaw=math.radians(25))))[1] == 0.0
    assert step(scene())[1] == 1.0


def test_tipped_cup_latches_zero():
    step(scene(cup=cup(*M.CUP_XY, tilt_deg=40)))
    assert M.TRIAL.mem["violated"] and "tea" in M.TRIAL.mem["why"]
    wipe_all()
    assert step(scene()) == (False, 0.0)


def test_robot_near_him_latches_zero():
    a, b, r = M.PERSON.capsules["R_forearm"]
    step(scene(), [R.gripper(b + [0, 0, r + 0.03]), R.PARKED[1]])
    assert M.TRIAL.mem["min_body"] < M.BODY_STANDOFF_M and M.TRIAL.mem["violated"]


def test_plans_reachable():
    for name in M.PLANS:
        for side, ws in M.PLANS[name]().items():
            assert R.reachable(side, ws), (name, side)


@pytest.mark.parametrize("name,want", [("demo", 1.0), ("naive", 0.0), ("unsafe", 0.0)])
def test_replays(name, want):
    got, mem = O.run(name)
    assert got == want, mem
    assert mem["wiped"] == 30 and not mem["violated"]
    assert mem["min_body"] > M.BODY_STANDOFF_M + 0.05
