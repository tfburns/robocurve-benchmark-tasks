"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhkit  # noqa: E402
import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
H = (M.CARTON / 2,) * 3


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def carton(x, y, z, quat=(0, 0, 0, 1), vel=(0, 0, 0)):
    return R.state((x, y, z), quat, half=H, vel=vel)


def on_mat(xy):
    return carton(*xy, M.MAT_TOP + M.CARTON / 2)


ON_TRAY = carton(*M.TRAY, M.TRAY_TOP + M.CARTON / 2)
DONE = {"carton_red": ON_TRAY, "carton_top": on_mat(M.SPOTS[0]), "carton_middle": on_mat(M.SPOTS[1])}


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def tilted(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def test_layout():
    assert set(TASK.free_objects) == set(M.CARTONS)
    s = scene()
    assert not any(M.fell(s[n]) for n in M.CARTONS) and not M.on_tray(s["carton_red"])
    gap = hhkit.poly_rect_gap(hhkit.footprint(s["carton_red"], (M.CARTON,) * 3),
                              hhkit.rect_of(M.AISLE, M.AISLE_SIZE[:2]))
    assert 0.04 <= gap <= 0.061  # the aisle is 4-6 cm from the stack, where a pulled-out stack lands
    mat_x0 = M.MAT[1] - M.MAT_SIZE[1] / 2
    assert mat_x0 > M.AISLE[1] + M.AISLE_SIZE[1] / 2  # mat and aisle do not overlap
    for xy in M.SPOTS:
        assert M._in_rect(np.array(xy), M.MAT, (M.MAT_SIZE[0] - M.CARTON, M.MAT_SIZE[1] - M.CARTON))


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_done_scores_one():
    assert run([(scene(**DONE), R.PARKED, NONE)] * 30) == 1.0


def test_others_stacked_on_the_mat_also_count():
    s = scene(carton_red=ON_TRAY, carton_middle=on_mat(M.SPOTS[0]),
              carton_top=carton(*M.SPOTS[0], M.MAT_TOP + 1.5 * M.CARTON + 0.0005))
    assert run([(s, R.PARKED, NONE)] * 30) == 1.0


def test_red_still_held_is_not_done():
    grip = [R.gripper(ON_TRAY.pos), R.PARKED[1]]
    assert run([(scene(**DONE), grip, NONE)] * 30) == 0.0


@pytest.mark.parametrize("bad, why", [
    (carton(0.24, 0.0, 0.9, vel=(0, 0, -1.0)), "fell"),
    (carton(0.24, 0.04, M.MAT_TOP + 0.03, tilted(60)), "tipped"),
    (carton(0.24, -0.075, TOP + M.CARTON / 2), "off the mat"),
    (carton(0.24, -0.10, M.MAT_TOP + 0.10), "aisle"),
])
def test_falls_latch_zero(bad, why):
    frames = [(scene(carton_top=bad), R.PARKED, NONE)] * 2 + [(scene(**DONE), R.PARKED, NONE)] * 40
    assert run(frames) == 0.0 and why in M.TRIAL.mem["why"]


def test_a_carton_lowered_slowly_is_not_a_fall():
    s = carton(0.24, 0.075, M.MAT_TOP + 0.05, vel=(0, 0, -0.2))
    assert M.fell(s) == ""


def test_reach():
    assert R.reachable("left", M.demo()["left"])


def test_demo_replay_scores_one():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert run(frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_demo_lifts_each_carton_straight_up_from_the_top_of_the_stack():
    order = [w.label for w in M.demo()["left"]].count("lift straight up")
    assert order == 3
