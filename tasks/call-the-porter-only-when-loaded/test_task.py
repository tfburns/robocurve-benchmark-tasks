"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

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
HALF = tuple(s / 2 for s in M.MEAL)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def meal(x, y, z=None):
    return R.state((x, y, M.PAD_TOP + M.MEAL[2] / 2 if z is None else z), half=HALF)


ON = {n: meal(*xy) for n, xy in M.SPOTS.items()}
ONE = {"meal_box_left": ON["meal_box_left"]}
PRESS = [R.PARKED[0], R.gripper((*M.HOUSING_C, M.CAP_TOP + 0.009))]
HOVER = [R.PARKED[0], R.gripper((*M.HOUSING_C, M.CAP_TOP + 0.03))]


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def test_layout():
    assert set(TASK.free_objects) == set(M.MEALS)
    for n, xy in M.SPOTS.items():
        assert M.on_pad(meal(*xy))
        assert not M.on_pad(scene()[n])
    gap = abs(M.SPOTS["meal_box_left"][1] - M.SPOTS["meal_box_right"][1]) - M.MEAL[1]
    assert gap >= 0.025  # room between the boxes on the pad
    assert not M.pressed(scene())


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 800) == 0.0


def test_ladder_without_the_call_stops_at_half():
    assert run([(scene(**ONE), R.PARKED, NONE)] * 30) == 0.25
    M.TRIAL.clear()
    assert run([(scene(**ON), R.PARKED, NONE)] * 30) == 0.5


def test_loaded_then_called_scores_one():
    frames = [(scene(**ON), R.PARKED, NONE)] * 5 + [(scene(**ON), PRESS, NONE)] * 5 + \
        [(scene(**ON), R.PARKED, NONE)] * 30
    assert run(frames) == 1.0 and M.TRIAL.mem["called"]


def test_calling_early_latches_zero_even_after_loading():
    frames = [(scene(**ONE), PRESS, NONE)] * 3 + [(scene(**ON), PRESS, NONE)] * 3 + \
        [(scene(**ON), R.PARKED, NONE)] * 30
    assert run(frames) == 0.0 and "1 meal box" in M.TRIAL.mem["why"]


def test_hovering_over_the_button_is_not_a_press():
    assert run([(scene(**ONE), HOVER, NONE)] * 10) == 0.25


def test_a_box_set_on_the_button_presses_it():
    on_button = meal(*M.HOUSING_C, M.CAP_TOP + M.MEAL[2] / 2 + 0.001)
    assert run([(scene(meal_box_right=on_button, **ONE), R.PARKED, NONE)] * 3) == 0.0


def test_a_box_hanging_over_the_pad_edge_does_not_count():
    edge = meal(M.PAD_C[0] + M.PAD[0] / 2, 0.05)
    assert not M.on_pad(edge)


def test_reach():
    d = M.demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_demo_replay_scores_one_within_the_run_limit():
    d = M.demo()
    assert max(sum(w.duration for w in ws) for ws in d.values()) <= 0.6 * 800 / R.HZ
    frames = R.replay(M, d, links=True, extra_s=2.0)
    assert run(frames) == 1.0 and M.TRIAL.mem["called"]


def test_calling_after_one_box_scores_zero():
    frames = R.replay(M, M.press_early_demo(), links=True, extra_s=2.0)
    assert run(frames) == 0.0 and M.TRIAL.mem["violated"]
