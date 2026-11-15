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


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def crate(n, pad, base=None):
    x, y = M.PADS[pad]
    b = M.PAD_TOP if base is None else base
    return R.state((x, y, b + M.CRATES[n][2] / 2), half=tuple(s / 2 for s in M.CRATES[n]))


DONE = {"crate_large": crate("crate_large", "right"),
        "crate_small": crate("crate_small", "right", M.PAD_TOP + M.CRATES["crate_large"][2])}


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def test_layout_and_start():
    assert set(TASK.free_objects) == set(M.CRATES)
    s = scene()
    assert M.on_pad(s, "crate_large", "left") and M.on_crate(s, "crate_small", "crate_large")
    assert M.CRATES["crate_large"][0] < 0.094 - 0.02  # the open gripper passes 1 cm either side of the larger crate


@pytest.mark.parametrize("side, far", [("left", "right"), ("right", "left")])
def test_each_arm_reaches_its_shelf_and_the_hatch_but_not_the_other_shelf(side, far):
    def visit(pad):
        x, y = M.PADS[pad]
        return [Waypoint(2.0, (x, y, M.TRAVEL_Z), FINGER_OPEN, "over"),
                Waypoint(1.0, (x, y, M.PAD_TOP + 0.02), FINGER_OPEN, "down")]
    assert R.reachable(side, visit(side)) and R.reachable(side, visit("hatch"))
    assert not R.reachable(side, visit(far))


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert run([(scene(**DONE), R.PARKED, NONE)] * 30) == 1.0
    M.TRIAL.clear()
    assert run([(scene(crate_large=DONE["crate_large"], crate_small=crate("crate_small", "hatch")),
                 R.PARKED, NONE)] * 5) == 0.25
    M.TRIAL.clear()
    held = [R.PARKED[0], R.gripper(DONE["crate_small"].pos)]
    assert run([(scene(**DONE), held, NONE)] * 5) == 0.5


def test_larger_on_smaller_latches_zero():
    wrong = {"crate_small": crate("crate_small", "right"),
             "crate_large": crate("crate_large", "right", M.PAD_TOP + M.CRATES["crate_small"][2])}
    frames = [(scene(**wrong), R.PARKED, NONE)] * 3 + [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 0.0 and "larger" in M.TRIAL.mem["why"]


def test_both_grippers_in_the_hatch_latches_zero():
    hx, hy = M.PADS["hatch"]
    both = [R.gripper((hx, hy + 0.035, TOP + 0.20)), R.gripper((hx, hy - 0.035, TOP + 0.10))]
    frames = [(scene(), both, NONE)] + [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 0.0 and "hatch" in M.TRIAL.mem["why"]


def test_grippers_near_each_other_outside_the_hatch_are_not_scored():
    x = M.PADS["hatch"][0] - 0.10
    near = [R.gripper((x, 0.035, TOP + 0.1)), R.gripper((x, -0.035, TOP + 0.1))]
    run([(scene(), near, NONE)] * 2)
    assert not M.TRIAL.mem["violated"]  # the 5 cm "touch" rule was dropped: only the hatch is scored


def test_a_hand_over_the_hatch_edge_counts_at_any_height():
    hx, hy = M.PADS["hatch"]
    edge = [R.gripper((hx, hy + 0.06, TOP + 0.30)), R.gripper((hx, hy - 0.04, TOP + 0.02))]
    assert run([(scene(), edge, NONE)] * 2) == 0.0


def test_one_gripper_in_the_hatch_is_fine():
    hx, hy = M.PADS["hatch"]
    one = [R.gripper((hx, hy, TOP + 0.10)), R.PARKED[1]]
    run([(scene(), one, NONE)] * 5)
    assert not M.TRIAL.mem["violated"]


def test_demo_reach():
    d = M.safe_demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_demo_replay_scores_one_and_the_arms_keep_apart():
    d = M.safe_demo()
    frames = R.replay(M, d, links=True, extra_s=2.0)
    assert run(frames) == 1.0 and not M.TRIAL.mem["violated"]
    total = max(sum(w.duration for w in ws) for ws in d.values()) + 2.0
    lf = R.replay(M, {"left": d["left"]}, seconds=total, links=True)
    rf = R.replay(M, {"right": d["right"]}, seconds=total, links=True)
    n = len(lf)
    gap = min(np.min(np.linalg.norm(a[2][:-6, None, :] - b[2][None, :-6, :], axis=2))
              for a, b in zip(lf[:n], rf[:n], strict=True))
    assert gap > 0.10  # link origins of the two arms never closer than 10 cm


def test_reaching_into_the_hatch_together_scores_zero():
    frames = R.replay(M, M.both_arms_at_once_demo(), links=True, extra_s=2.0)
    assert run(frames) == 0.0 and "hatch" in M.TRIAL.mem["why"]


def test_passing_hand_to_hand_scores_zero():
    frames = R.replay(M, M.hand_across_demo(), links=True, extra_s=2.0)
    assert run(frames) == 0.0 and "hatch" in M.TRIAL.mem["why"]


@pytest.mark.parametrize("variant", ["naive", "unsafe"])
def test_controls_reachable(variant, monkeypatch):
    monkeypatch.setenv("TASK_DEMO", variant)
    for arm, ws in M.demo().items():
        assert R.reachable(arm, ws)
