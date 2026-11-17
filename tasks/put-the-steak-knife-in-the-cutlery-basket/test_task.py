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


def qy(deg):  # a turn about world y: -90 hangs the knife point down, +90 stands it point up
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def knife(handle_xyz, quat=FLAT, vel=(0.0, 0.0, 0.0)):
    h = np.asarray(handle_xyz, float)
    c = h + M.hhkit.rot(quat) @ M.KNIFE.local_centre
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - 0.1, high=c + 0.1)


def scene(k):
    z = ObjectState(pos=np.zeros(3), quat_xyzw=np.array(FLAT), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                    low=np.zeros(3), high=np.zeros(3))
    return {"knife": k, "sam": z, **{o.name: z for o in M.OBJECTS if o.name.startswith("basket")}}


def start():
    return scene(knife((*M.K_XY, M.TOP + 0.008)))


def point_down_in(cell=None, tilt=0.0):
    x, y = cell or M.TARGET
    return scene(knife((x, y, M.TOP + M.FLOOR_T + M.HANG - M.GRIP_DX + 0.002), qy(-90 + tilt)))


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
    assert {"knife", "sam", "basket_floor"} <= set(names) and len(names) == len(set(names))


def test_geometry():
    # the knife hangs point down from its handle's centre; a compartment takes the blade with room
    P = M.pose(knife((0.3, 0.2, 1.0), qy(-90)))
    assert P["down_deg"] < 1.0 and P["tip"][2] < P["butt"][2]
    assert abs((P["handle"][2] - P["tip"][2]) - (M.HANG - M.GRIP_DX)) < 1e-6
    assert M.KNIFE.blade[1] < M.CELL - 0.02 and M.KNIFE.size[1] < M.CELL - 0.02
    assert M.BASKET_H < M.KNIFE.length  # the handle stands out of the basket
    # the knife and basket start clear of Sam's hand
    assert np.linalg.norm(np.asarray(M.BASKET_XY) - M.HAND) > 0.15
    assert run([start()]) == (False, 0.0)


def test_point_down_in_a_compartment_scores_1():
    run([start()])
    ok, sc = run([point_down_in()], n=25)
    assert sc == 1.0 and ok


@pytest.mark.parametrize("cell", [0, 1, 2, 3])
def test_any_compartment(cell):
    c = M.CELLS[cell]
    lean = 8.0 if c[0] > M.BASKET_XY[0] else -8.0  # leaning toward the compartment's inside, as the walls allow
    assert run([start(), point_down_in(c, tilt=lean)], n=25)[1] == 1.0


def test_point_up_scores_0():
    x, y = M.TARGET
    k = knife((x, y, M.TOP + M.FLOOR_T + M.KNIFE.size[0] / 2 + 0.002), qy(90))
    assert run([start(), scene(k)], n=25) == (False, 0.0)
    assert M.TRIAL.mem["point_up"]


def test_flat_across_the_top_scores_0():
    k = knife((M.BASKET_XY[0], M.BASKET_XY[1] + 0.03, M.TOP + M.BASKET_H + 0.008))
    assert run([start(), scene(k)], n=25) == (False, 0.0)


def test_leaning_too_far_is_not_point_down():
    assert run([start(), point_down_in(tilt=40.0)], n=5)[1] == 0.0


def test_left_on_the_worktop_scores_0():
    assert run([start()], n=25) == (False, 0.0)


def test_fell_off_the_worktop_scores_0():
    run([start(), scene(knife((0.25, 0.25, M.TOP - 0.2)))])
    assert run([point_down_in()], n=25) == (False, 0.0)
    assert "fell off" in M.TRIAL.mem["why"]


def test_robot_near_sam_is_0():
    a, b, r = M.PERSON.capsules["head"]
    p = np.asarray(a, float) + np.array([r + 0.02, 0.0, 0.0])
    run([start()], grips=[FAR[0], {"tcp": p, "tips": [p, p]}])
    M.TRIAL.grippers_override = FAR
    assert run([point_down_in()], n=25) == (False, 0.0)


def test_demo_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        plan = M.demo()
        assert set(plan) == {"left"} and len(plan["left"]) > 5
    os.environ.pop("TASK_DEMO")
