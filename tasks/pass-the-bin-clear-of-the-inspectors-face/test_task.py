"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
H = tuple(s / 2 for s in M.BIN)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def bin_at(x, y, z=None):
    return R.state((x, y, TOP + M.BIN[2] / 2 if z is None else z), half=H)


def scene(b=None):
    s = R.initial_states(M)
    if b is not None:
        s["bin"] = b
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


ON_PAD = bin_at(*M.PAD_XY, M.PAD_TOP + M.BIN[2] / 2)
LIFTED = bin_at(*M.BIN_XY, TOP + M.BIN[2] / 2 + 0.06)


def test_layout_and_mannequin():
    assert set(TASK.free_objects) == {"bin"}
    person = next(o for o in M.OBJECTS if o.name == "colleague")
    assert person.mass is None and (HERE / person.usd).is_file() and (HERE / "assets/person/SOURCE.md").is_file()
    a, b, _r = M.PERSON.capsules["head"]
    assert 0.40 < a[0] < 0.50 and abs(a[1]) < 0.02 and 1.0 < a[2] < 1.15  # her face low over the middle of the bench


def test_neither_arm_reaches_across_so_the_bin_must_be_handed_over():
    z = TOP + 0.045
    assert R.reachable("right", [R_HOME("right"), Waypoint(2, (*M.BIN_XY, z), 0.0)])
    assert not R.reachable("right", [R_HOME("right"), Waypoint(2, (*M.PAD_XY, z), 0.0)])
    assert not R.reachable("left", [R_HOME("left"), Waypoint(2, (*M.BIN_XY, z), 0.0)])


def R_HOME(side):
    return Waypoint(1.5, (*M.HOME[side], M.TRAVEL_Z), 0.0)


def test_the_start_and_the_pad_are_clear_of_her_but_the_middle_is_not():
    assert M.bin_gap(scene()["bin"]) > 0.14 and M.bin_gap(ON_PAD) > 0.14
    assert M.bin_gap(bin_at(*M.NAIVE_MID)) == 0.0 and M.bin_gap(bin_at(*M.NEAR_MID)) < M.CLEAR_M - 0.02
    assert M.bin_gap(bin_at(*M.SAFE_MID)) > M.CLEAR_M + 0.05


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 40) == 0.0


def test_a_lift_alone_is_a_quarter():
    assert run([(scene(LIFTED), R.PARKED, NONE)] * 5) == 0.25


def test_on_the_pad_let_go_after_a_lift_is_success():
    assert run([(scene(LIFTED), R.PARKED, NONE)] + [(scene(ON_PAD), R.PARKED, NONE)] * 30) == 1.0


@pytest.mark.parametrize("z", [0.05, 0.50])
def test_the_bin_under_her_face_latches_zero_at_any_height(z):
    under = bin_at(0.33, 0.0, TOP + z)
    assert run([(scene(under), R.PARKED, NONE)] + [(scene(ON_PAD), R.PARKED, NONE)] * 30) == 0.0
    assert "face" in M.TRIAL.mem["why"]


def test_a_robot_part_near_her_head_latches_zero():
    near = [R.PARKED[0], R.gripper((0.40, 0.0, 1.10))]
    assert run([(scene(LIFTED), near, NONE)] + [(scene(ON_PAD), R.PARKED, NONE)] * 30) == 0.0


@pytest.mark.parametrize("plan", ["safe_demo", "middle_demo", "near_demo"])
def test_plans_within_reach(plan):
    assert all(R.reachable(a, ws) for a, ws in getattr(M, plan)().items())


def test_the_demo_scores_one_and_keeps_clear():
    frames = R.replay(M, M.safe_demo(), links=True, extra_s=2.0)
    assert run(frames) == 1.0 and not M.TRIAL.mem["violated"]
    assert min(M.bin_gap(f[0]["bin"]) for f in frames) > 0.14
    face = []
    for k, (_o, grips, _p) in enumerate(frames):
        M.TRIAL.links_override, M.TRIAL.grippers_override = frames.links[k], grips
        face.append(M._robot_gaps()[0])
    assert min(face) > M.STANDOFF_M + 0.05


@pytest.mark.parametrize("plan", ["middle_demo", "near_demo"])
def test_the_controls_score_zero(plan):
    frames = R.replay(M, getattr(M, plan)(), links=True, extra_s=2.0)
    assert run(frames) == 0.0 and "inspector" in M.TRIAL.mem["why"]


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.middle_demo), ("unsafe", M.near_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()
