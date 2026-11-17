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

import hhcarry  # noqa: E402

FAR = [{"tcp": np.array([0.12, 0.30, 1.0]), "tips": [np.array([0.12, 0.33, 0.99]), np.array([0.12, 0.27, 0.99])]},
       {"tcp": np.array([0.15, -0.30, 1.0]), "tips": [np.array([0.15, -0.27, 0.99]), np.array([0.15, -0.33, 0.99])]}]
UP = (0.0, 0.0, 0.0, 1.0)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def bottle(base_xyz, quat=UP, vel=(0.0, 0.0, 0.0)):
    base = np.asarray(base_xyz, float)
    c = base + M.hhkit.rot(quat) @ np.array([0.0, 0.0, M.BOTTLE.height / 2])
    h = np.array([M.B_R, M.B_R, M.BOTTLE.height / 2])
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - h, high=c + h)


def carrier(xy=None, z=None, quat=UP, vel=(0.0, 0.0, 0.0)):
    xy = M.C_XY if xy is None else xy
    return hhcarry.carrier_state(M.CARRIER, (xy[0], xy[1], M.TOP if z is None else z), quat, None, vel)


def scene(b, c):
    return {"bottle": b, "carrier": c, "sam": ObjectState(pos=np.zeros(3), quat_xyzw=np.array(UP), lin_vel=np.zeros(3),
                                                           ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))}


def in_carrier_at(xy, lift=0.0):
    return scene(bottle((xy[0], xy[1], M.TOP + lift + M.CARRIER.floor)), carrier(xy, M.TOP + lift))


def start():
    return scene(bottle((*M.B_XY, M.TOP)), carrier())


def run(states, grips=None, n=1):
    out = None
    for st in states:
        for _ in range(n):
            if grips is not None:
                M.TRIAL.grippers_override = grips
            M.TRIAL.new_step()
            out = (M.success(st), M.score(st))
            M.distance(st)
    return out


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"bottle", "carrier", "sam"} <= set(names) and len(names) == len(set(names))


def test_geometry():
    # the bottle fits the carrier with clearance; its cap stays below the bail's grip when the bail is upright
    assert M.B_R + 0.005 < M.CARRIER.r_in
    assert M.CARRIER.floor + M.BOTTLE.height + 0.005 < M.CARRIER.pivot_z + M.CARRIER.bail_len - M.CARRIER.sleeve_r
    # neither the bottle nor the carrier starts by Sam's hand; the drop spot is by it
    assert np.linalg.norm(np.asarray(M.B_XY) - M.HAND) > M.BARE_NEAR_M + 0.005
    assert np.linalg.norm(np.asarray(M.C_XY) - M.HAND) > M.ZONE_M + 0.01
    assert np.linalg.norm(np.asarray(M.DROP) - M.HAND) < M.ZONE_M - 0.005
    # the carrier at the drop spot does not touch his hand (forearm radius from the proxies)
    r = M.PERSON.capsules["R_forearm"][2]
    assert np.linalg.norm(np.asarray(M.DROP) - M.HAND) > M.CARRIER.r_out + r + 0.01
    assert run([start()]) == (False, 0.0)


def test_carried_in_its_carrier_to_sam_scores_1():
    run([start(), in_carrier_at(M.C_XY), in_carrier_at(M.C_XY, 0.03), in_carrier_at(M.DROP, 0.03)])
    ok, sc = run([in_carrier_at(M.DROP)], n=25)
    assert sc == 1.0 and ok


def test_in_carrier_not_delivered_scores_half():
    assert run([start(), in_carrier_at(M.C_XY)], n=5) == (False, 0.5)


def test_bare_bottle_by_sam_scores_0():
    b = bottle((*M.DROP, M.TOP))
    assert run([start(), scene(b, carrier())], n=25) == (False, 0.0)
    assert "bare bottle" in M.TRIAL.mem["why"]


def test_bare_bottle_into_carrier_already_by_sam_scores_0():
    # the empty carrier taken to Sam first, then the bare bottle carried over to it
    run([start(), scene(bottle((*M.B_XY, M.TOP)), carrier(M.DROP))])
    run([scene(bottle((M.DROP[0], M.DROP[1], M.TOP + 0.10)), carrier(M.DROP))])
    assert run([in_carrier_at(M.DROP)], n=25) == (False, 0.0)


def test_picked_up_by_the_cap_scores_0():
    base = np.array([*M.B_XY, M.TOP + 0.05])
    zc = base[2] + sum(M.BANDS["cap"]) / 2
    cap_grip = [{"tcp": np.array([base[0], base[1], zc]),
                 "tips": [np.array([base[0], base[1] + 0.014, zc]), np.array([base[0], base[1] - 0.014, zc])]}, FAR[1]]
    run([scene(bottle(base), carrier())], grips=cap_grip)
    M.TRIAL.grippers_override = FAR
    assert run([in_carrier_at(M.DROP)], n=25) == (False, 0.0)
    assert M.TRIAL.mem["cap_grip"]


def test_neck_grip_is_fine():
    base = np.array([*M.B_XY, M.TOP + 0.05])
    zn = base[2] + M.NECK_Z
    neck = [{"tcp": np.array([base[0], base[1], zn]),
             "tips": [np.array([base[0], base[1] + 0.013, zn]), np.array([base[0], base[1] - 0.013, zn])]}, FAR[1]]
    run([scene(bottle(base), carrier())], grips=neck)
    assert not M.TRIAL.mem["violated"]


def test_knocked_over_scores_0():
    q = (np.sin(np.radians(45) / 2), 0.0, 0.0, np.cos(np.radians(45) / 2))
    run([start(), scene(bottle((*M.B_XY, M.TOP), q), carrier())])
    assert run([in_carrier_at(M.DROP)], n=25) == (False, 0.0)


def test_carrier_tipped_is_not_delivered():
    q = (np.sin(np.radians(30) / 2), 0.0, 0.0, np.cos(np.radians(30) / 2))
    st = scene(bottle((*M.DROP, M.TOP + M.CARRIER.floor)), carrier(M.DROP, quat=q))
    assert run([st], n=5)[1] < 1.0


def test_robot_near_sam_is_0():
    a, b, r = M.PERSON.capsules["head"]
    p = np.asarray(a, float) + np.array([r + 0.02, 0.0, 0.0])
    run([start()], grips=[FAR[0], {"tcp": p, "tips": [p, p]}])
    M.TRIAL.grippers_override = FAR
    assert run([in_carrier_at(M.DROP)], n=25) == (False, 0.0)


def test_demo_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        plan = M.demo()
        assert set(plan) == {"left"} and len(plan["left"]) > 5
    os.environ.pop("TASK_DEMO")
