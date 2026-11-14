"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
H = tuple(s / 2 for s in M.PART)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def part_on(cell):
    return R.state((*M.CELLS[cell], M.PAD_TOP + M.PART[2] / 2), half=H)


DONE = {"part_red": part_on("right"), "part_blue": part_on("left")}


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def test_layout():
    assert set(TASK.free_objects) == set(M.PARTS)
    s = scene()
    assert M.on_cell(s["part_red"], "left") and M.on_cell(s["part_blue"], "right")
    for xy in M.SPOTS.values():
        assert abs(xy[1]) + M.PART[1] / 2 <= M.STRIP_W / 2  # the strip spots lie inside the strip
    assert abs(M.SPOTS["part_red"][0] - M.SPOTS["part_blue"][0]) - M.PART[0] >= 0.04


@pytest.mark.parametrize("side, far", [("left", "right"), ("right", "left")])
def test_each_arm_reaches_its_cell_and_the_strip_but_not_the_other_cell(side, far):
    def visit(xy):
        return [Waypoint(2.0, (*xy, M.TRAVEL_Z), FINGER_OPEN, "over"),
                Waypoint(1.0, (*xy, M.PAD_TOP + 0.02), FINGER_OPEN, "down")]
    assert R.reachable(side, visit(M.CELLS[side]))
    assert all(R.reachable(side, visit(xy)) for xy in M.SPOTS.values())
    assert not R.reachable(side, visit(M.CELLS[far]))


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert run([(scene(**DONE), R.PARKED, NONE)] * 30) == 1.0
    M.TRIAL.clear()
    assert run([(scene(part_red=DONE["part_red"]), R.PARKED, NONE)] * 5) == 0.5
    M.TRIAL.clear()
    held = [R.gripper(DONE["part_blue"].pos), R.PARKED[1]]
    assert run([(scene(**DONE), held, NONE)] * 5) == 0.5


def test_both_in_the_strip_latches_zero():
    both = [R.gripper((0.25, 0.04, TOP + 0.15)), R.gripper((0.15, -0.04, TOP + 0.10))]
    frames = [(scene(), both, NONE)] + [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 0.0 and "strip" in M.TRIAL.mem["why"]


def test_touching_latches_zero():
    near = [R.gripper((0.10, 0.09, TOP + 0.1)), R.gripper((0.10, 0.07, TOP + 0.1))]
    assert run([(scene(), near, NONE)] * 2) == 0.0 and "touched" in M.TRIAL.mem["why"]


def test_one_arm_in_the_strip_is_fine():
    one = [R.gripper((0.22, 0.0, TOP + 0.1)), R.PARKED[1]]
    run([(scene(), one, NONE)] * 5)
    assert not M.TRIAL.mem["violated"]


def test_reach():
    d = M.demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_demo_replay_scores_one_and_the_arms_keep_apart():
    d = M.demo()
    frames = R.replay(M, d, links=True, extra_s=2.0)
    assert run(frames) == 1.0 and not M.TRIAL.mem["violated"]
    total = max(sum(w.duration for w in ws) for ws in d.values()) + 2.0
    lf = R.replay(M, {"left": d["left"]}, seconds=total, links=True)
    rf = R.replay(M, {"right": d["right"]}, seconds=total, links=True)
    gap = min(np.min(np.linalg.norm(a[2][:-6, None, :] - b[2][None, :-6, :], axis=2))
              for a, b in zip(lf, rf, strict=True))
    assert gap > 0.10


def test_starting_early_puts_both_arms_in_the_strip():
    frames = R.replay(M, M.both_at_once_demo(), links=True, extra_s=2.0)
    assert run(frames) == 0.0 and "strip" in M.TRIAL.mem["why"]
