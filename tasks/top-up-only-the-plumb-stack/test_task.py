"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
H = tuple(s / 2 for s in M.BOX)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def box(x, y, z, quat=(0, 0, 0, 1)):
    return R.state((x, y, z), quat, half=H)


def on_top(stack_top_name):
    x, y, _ = M.STACKED[stack_top_name]
    return box(x, y, M._layer_z(M.LAYERS))


IN_TOTE = box(*M.TOTE_C, TOP + 0.005 + M.BOX[2] / 2)
DONE = {"loose_box_right": on_top(M.PLUMB_TOP), "loose_box_left": IN_TOTE}


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def settle():
    return [(scene(), R.PARKED, NONE)] * int(M.SETTLE_S * R.HZ + 2)


def stable(xs, extra=None):
    """Static check along x for a column of boxes at centres xs (bottom first), plus a full loose box on top at x
    extra: the centre of mass of everything above each box lies over that box."""
    ms = [M.BOX_KG] * len(xs)
    xs = list(xs)
    if extra is not None:
        xs.append(extra)
        ms.append(M.LOOSE_KG)
    for k in range(len(xs) - 1):
        com = sum(m * x for m, x in zip(ms[k + 1:], xs[k + 1:], strict=True)) / sum(ms[k + 1:])
        if abs(com - xs[k]) > M.BOX[0] / 2:
            return False
    return True


def test_the_leaning_stack_stands_but_one_more_box_on_top_brings_it_down():
    xs = [M.LEANING[f"leaning_{k + 1}"][0] for k in range(M.LAYERS)]
    assert stable(xs)
    margin = M.BOX[0] / 2 - (sum(xs[1:]) / 3 - xs[0])
    assert margin >= 0.004  # 4 mm from tipping as it stands
    assert not stable(xs, extra=xs[-1])  # a full box lined up with the top one (or further out) tips it
    plumb = [M.PLUMB[f"plumb_{k + 1}"][0] for k in range(M.LAYERS)]
    assert stable(plumb, extra=plumb[-1] + 0.02)  # the plumb stack takes a box even 2 cm off centre


def test_layout():
    assert set(TASK.free_objects) == set(M.STACKED) | set(M.LOOSE)
    s = scene()
    assert not any(M.in_tote(s[n]) for n in M.LOOSE)
    for n in M.LOOSE:  # loose boxes clear of the tote walls
        assert abs(s[n].pos[1]) - M.BOX[1] / 2 > M.TOTE_IN[1] / 2 + M.TOTE_WALL
    assert M.BOX[0] <= M.TOTE_IN[0] - 0.015 and M.BOX[1] <= M.TOTE_IN[1] - 0.015


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert run(settle() + [(scene(**DONE), R.PARKED, NONE)] * 30) == 1.0
    M.TRIAL.clear()
    assert run(settle() + [(scene(loose_box_left=IN_TOTE), R.PARKED, NONE)] * 5) == 0.5
    M.TRIAL.clear()
    assert run(settle() + [(scene(loose_box_right=on_top(M.PLUMB_TOP)), R.PARKED, NONE)] * 5) == 0.5


def test_both_boxes_in_the_tote_is_half():
    other = box(M.TOTE_C[0], M.TOTE_C[1], TOP + 0.005 + 1.5 * M.BOX[2])
    assert run(settle() + [(scene(loose_box_left=IN_TOTE, loose_box_right=other), R.PARKED, NONE)] * 5) == 0.5


def test_a_box_on_the_leaning_stack_latches_zero():
    frames = settle() + [(scene(loose_box_left=on_top(M.LEAN_TOP)), R.PARKED, NONE)] * 3 + \
        [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 0.0 and "leaning" in M.TRIAL.mem["why"]


def test_a_toppled_box_latches_zero():
    x, y, _ = M.LEANING[M.LEAN_TOP]
    a = math.radians(40) / 2
    fallen = box(x + 0.06, y, TOP + 0.03, (0, math.sin(a), 0, math.cos(a)))
    frames = settle() + [(scene(**{M.LEAN_TOP: fallen}), R.PARKED, NONE)] * 3
    assert run(frames) == 0.0 and "toppled" in M.TRIAL.mem["why"]


def test_a_held_box_does_not_count():
    grip = [R.PARKED[0], R.gripper(DONE["loose_box_right"].pos)]
    assert run(settle() + [(scene(**DONE), grip, NONE)] * 5) == 0.5


def test_reach():
    d = M.demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])
    assert R.reachable("left", M.load_both_demo()["left"])


def test_demo_replay_scores_one():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert run(frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_loading_the_leaning_stack_scores_zero():
    frames = R.replay(M, M.load_both_demo(), links=True, extra_s=2.0)
    assert run(frames) == 0.0 and "leaning" in M.TRIAL.mem["why"]
