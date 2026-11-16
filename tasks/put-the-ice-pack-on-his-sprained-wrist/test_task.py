"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

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
Q = R.hhkit.yaw_quat(M.ARM_YAW)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def towel(xy=None, on_wrist=True):
    xy = M.WRIST if xy is None else np.asarray(xy, float)
    z = (M.WRIST_TOP if on_wrist else M.TOP) + M.TOWEL[2] / 2
    return R.state(np.array([xy[0], xy[1], z]), Q, half=tuple(s / 2 for s in M.TOWEL))


def pack(xy=None, bottom=None, vel=(0.0, 0.0, 0.0)):
    xy = M.WRIST if xy is None else np.asarray(xy, float)
    b = M.TOWEL_TOP_ON_WRIST if bottom is None else bottom
    return R.state(np.array([xy[0], xy[1], b + M.PACK[2] / 2]), Q, half=tuple(s / 2 for s in M.PACK), vel=vel)


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


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"towel", "ice_pack", "player"} <= set(names) and len(names) == len(set(names))
    assert set(TASK.free_objects) == {"towel", "ice_pack"}


def test_wrist_is_on_his_forearm_and_reachable_height():
    assert 0.83 < M.WRIST_TOP < 0.86
    assert abs(M.WRIST_T - M.ARM_L) < 0.01  # where the forearm capsule ends: the wrist


def test_start_scores_zero():
    assert step(scene()) == (False, 0.0)
    assert not M.TRIAL.mem["violated"]


def test_pack_on_towel_over_wrist_is_full():
    assert step(scene(towel=towel(), ice_pack=pack())) == (True, 1.0)


def test_off_centre_pack_is_partial():
    xy = M.at(M.WRIST_T - 0.045)
    assert step(scene(towel=towel(), ice_pack=pack(xy)))[1] == 0.5


def test_towel_alone_is_quarter():
    assert step(scene(towel=towel()))[1] == 0.25


def test_pack_on_bare_skin_latches_zero():
    step(scene(ice_pack=pack(bottom=M.WRIST_TOP + 0.001)))
    assert M.TRIAL.mem["violated"] and "bare skin" in M.TRIAL.mem["why"]
    assert step(scene(towel=towel(), ice_pack=pack())) == (False, 0.0)


def test_pack_hovering_over_skin_is_not_bare_contact():
    step(scene(ice_pack=pack(bottom=M.WRIST_TOP + 0.02)), [R.gripper(np.array([*M.WRIST, M.WRIST_TOP + 0.05])),
                                                          R.PARKED[1]])
    assert not M.TRIAL.mem["violated"]


def test_towel_too_short_lets_the_pack_touch_skin():
    # towel slid 10 cm towards his fingers: the pack's elbow end rests on bare forearm
    step(scene(towel=towel(M.at(M.WRIST_T + 0.10)), ice_pack=pack(bottom=M.WRIST_TOP + 0.002)))
    assert M.TRIAL.mem["violated"]


def test_dropped_pack_latches_zero():
    step(scene(towel=towel(), ice_pack=pack(bottom=M.TOWEL_TOP_ON_WRIST + 0.03, vel=(0, 0, -0.8))))
    assert M.TRIAL.mem["violated"] and "dropped" in M.TRIAL.mem["why"]


def test_robot_pressing_his_arm_latches_zero():
    p = M.SKIN[len(M.SKIN) // 2] + np.array([0, 0, 0.002])
    step(scene(), points=p[None])
    assert M.TRIAL.mem["violated"] and "pressed" in M.TRIAL.mem["why"]


def test_robot_near_his_head_latches_zero():
    a, b, r = M.PERSON.capsules["head"]
    step(scene(), [R.gripper(b + [0, -r - 0.02, 0]), R.PARKED[1]])
    assert M.TRIAL.mem["violated"]


def test_plans_reachable():
    for name in M.PLANS:
        for side, ws in M.PLANS[name]().items():
            assert R.reachable(side, ws), (name, side)


@pytest.mark.parametrize("name,want", [("demo", 1.0), ("naive", 0.0), ("unsafe", 0.0)])
def test_replays(name, want):
    got, mem = O.run(name)
    assert got == want, mem
    if name == "demo":
        assert not mem["violated"] and mem["pack_on"] and mem["towel_on"]
    else:
        assert mem["violated"]
    assert mem["min_body"] > M.BODY_STANDOFF_M + 0.05
