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


def jar(base_xyz, quat=UP, vel=(0.0, 0.0, 0.0)):
    b = np.asarray(base_xyz, float)
    c = b + M.hhkit.rot(quat) @ np.array([0.0, 0.0, M.JAR.height / 2])
    half = np.array([0.0335, 0.0335, M.JAR.height / 2])
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - half, high=c + half)


ON_RACK = (*M.JAR_XY, M.TOP + M.RACK_SIZE[2])
OTHER = (*M.OTHER_XY, M.TOWEL_TOP)
ON_TOWEL = (*M.DROP_XY, M.TOWEL_TOP)


def scene(j, o=None):
    return {"jar": j, "cooling_jar": o if o is not None else jar(OTHER)}


def start():
    return scene(jar(ON_RACK))


def carried(xy=(0.30, 0.20), z=0.08):
    return jar((*xy, M.TOP + z))


def grip_at(j_base, z_above):
    t = np.array([j_base[0], j_base[1], j_base[2] + z_above])
    return [{"tcp": t, "tips": [t + [0, 0.03, 0], t - [0, 0.03, 0]]}, FAR[1]]


def run(states, n=1, grips=None):
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
    assert {"jar", "cooling_jar", "rack", "towel"} <= set(names) and len(names) == len(set(names))


def test_geometry():
    # the drop spot is on the towel and leaves more than an inch to the cooling jar; the jars start apart
    assert M._in_rect(ON_TOWEL, M.TOWEL_XY, M.TOWEL_SIZE, shrink=0.01)
    assert M._in_rect(OTHER, M.TOWEL_XY, M.TOWEL_SIZE)
    assert np.linalg.norm(np.subtract(M.DROP_XY, M.OTHER_XY)) - 2 * M.J_R > M.GAP_MIN_M + 0.01
    # the rack, the towel and the naive spot do not overlap
    assert abs(M.RACK_XY[1] - M.TOWEL_XY[1]) > (M.RACK_SIZE[1] + M.TOWEL_SIZE[1]) / 2
    assert not M._in_rect(M.STEEL_XY, M.RACK_XY, M.RACK_SIZE, shrink=-M.J_R)
    assert not M._in_rect(M.STEEL_XY, M.TOWEL_XY, M.TOWEL_SIZE, shrink=-M.J_R)
    # the body grip is below the neck, the band grip in the band
    assert M.Z_BODY + 0.01 < M.BANDS["neck"][0] and M.BANDS["band"][0] < M.Z_BAND < M.BANDS["band"][1]
    assert run([start()]) == (False, 0.0)


def test_onto_towel_scores_1():
    run([start(), scene(carried(), jar(OTHER))])
    ok, sc = run([scene(jar(ON_TOWEL))], n=25)
    assert sc == 1.0 and ok


def test_on_rack_is_not_done():
    assert run([start()], n=25) == (False, 0.0)


def test_set_on_steel_is_0_even_if_moved_on():
    run([start(), scene(jar((*M.STEEL_XY, M.TOP)))], n=3)
    assert M.TRIAL.mem["violated"] and "cold steel" in M.TRIAL.mem["why"]
    assert run([scene(jar(ON_TOWEL))], n=25) == (False, 0.0)


def test_passing_low_over_the_bench_while_moving_is_not_a_set_down():
    run([start(), scene(jar((*M.STEEL_XY, M.TOP + 0.002), vel=(0.2, 0.0, 0.0)))])
    assert not M.TRIAL.mem["violated"]


def test_lifted_by_band_is_0():
    b = (0.30, 0.20, M.TOP + 0.08)
    run([start(), scene(jar(b))], grips=grip_at(b, M.Z_BAND))
    M.TRIAL.grippers_override = FAR
    assert run([scene(jar(ON_TOWEL))], n=25) == (False, 0.0)
    assert "screw band" in M.TRIAL.mem["why"]


def test_lifted_by_glass_is_fine():
    b = (0.30, 0.20, M.TOP + 0.08)
    run([start(), scene(jar(b))], grips=grip_at(b, M.Z_BODY))
    M.TRIAL.grippers_override = FAR
    ok, sc = run([scene(jar(ON_TOWEL))], n=25)
    assert sc == 1.0 and ok


def test_band_at_rest_on_rack_is_not_a_lift():
    run([start()], grips=grip_at(ON_RACK, M.Z_BAND))
    assert not M.TRIAL.mem["violated"]


def test_tilted_is_0():
    q = (np.sin(np.radians(25) / 2), 0.0, 0.0, np.cos(np.radians(25) / 2))
    run([start(), scene(jar((0.30, 0.20, M.TOP + 0.08), q))])
    assert run([scene(jar(ON_TOWEL))], n=25) == (False, 0.0)
    assert "tilted" in M.TRIAL.mem["why"]


def test_too_close_to_the_other_jar_is_0():
    near = (M.OTHER_XY[0] - 0.075, M.OTHER_XY[1] + 0.02, M.TOWEL_TOP)  # 1.6 cm of glass between them
    assert M.jar_gap(jar(near), jar(OTHER)) < M.GAP_MIN_M
    assert run([start(), scene(jar(near))], n=25) == (False, 0.0)


def test_knocking_the_cooling_jar_is_0():
    run([start(), scene(jar(ON_RACK), jar((M.OTHER_XY[0] + 0.03, M.OTHER_XY[1], M.TOWEL_TOP)))])
    assert "knocked" in M.TRIAL.mem["why"]


def test_jar_off_bench_is_0():
    run([start(), scene(jar((0.3, 0.0, M.TOP - 0.2)))])
    assert run([scene(jar(ON_TOWEL))], n=25) == (False, 0.0)


def test_demo_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        plan = M.demo()
        assert set(plan) == {"left"} and len(plan["left"]) > 5
    os.environ.pop("TASK_DEMO")
