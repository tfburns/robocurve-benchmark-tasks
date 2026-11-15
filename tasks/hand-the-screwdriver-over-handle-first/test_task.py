"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
H = tuple(s / 2 for s in M.TOOL)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def tool_at(x, y, yaw, z=None):
    return R.state((x, y, M.TRAY_TOP + M.TOOL[2] / 2 if z is None else z), R.hhkit.yaw_quat(yaw), half=H)


def scene(tool=None):
    s = R.initial_states(M)
    if tool is not None:
        s["screwdriver"] = tool
    return s


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


LIFTED = tool_at(*M.TOOL_XY, 0.0, TOP + 0.12)


def test_loads_with_a_loose_screwdriver_drawn_as_handle_and_shaft_and_a_fixed_mannequin():
    assert set(TASK.free_objects) == {"screwdriver"}
    tool = next(o for o in M.OBJECTS if o.name == "screwdriver")
    trim = {t[0]: t for t in tool.trim}
    assert trim["steel_shaft"][2][0] > 0 > trim["handle"][2][0]
    person = next(o for o in M.OBJECTS if o.name == "technician")
    assert person.mass is None and (HERE / person.usd).is_file()
    assert (HERE / "assets/person/SOURCE.md").is_file() and (HERE / "assets/person/LICENSE-Rocketbox.md").is_file()


def test_her_right_hand_rests_on_the_bench_just_past_the_tray():
    pts = M.PERSON.points["R_hand"]
    assert TOP < pts[:, 2].min() < TOP + 0.012
    assert M.TRAY_XY[0] + M.TRAY[0] / 2 < pts[:, 0].min() < M.TRAY_XY[0] + M.TRAY[0] / 2 + 0.03
    assert abs(float(M.HAND_XY[1])) < 0.03


def test_the_start_is_clear_and_scores_zero():
    s = scene()["screwdriver"]
    assert M.tip_gap(s) > 0.1
    assert step(scene()) == (False, 0.0)


@pytest.mark.parametrize("yaw_deg, ok", [(0, False), (90, False), (-90, False), (180, True), (-143, True)])
def test_which_directions_count_as_handed_over(yaw_deg, ok):
    assert M.handed_over(tool_at(*M.PLACE_XY, math.radians(yaw_deg))) == ok


def test_in_the_tray_handle_first_after_a_lift_is_success():
    step(scene(LIFTED))
    assert step(scene(tool_at(*M.PLACE_XY, M.TURN))) == (True, 1.0)


def test_put_down_shaft_first_latches_zero():
    step(scene(LIFTED))
    assert step(scene(tool_at(0.12, -0.03, 0.0)))[1] == 0.0
    assert "pointing at" in M.TRIAL.mem["why"]
    assert step(scene(tool_at(*M.PLACE_XY, M.TURN)))[1] == 0.0


def test_held_shaft_first_away_from_her_is_not_a_put_down():
    s = tool_at(0.10, -0.03, 0.0)
    step(scene(LIFTED))
    step(scene(s), [R.gripper(s.pos), R.PARKED[1]])
    assert not M.TRIAL.mem["violated"]


@pytest.mark.parametrize("z", [0.03, 0.10])
def test_the_tip_near_her_hand_latches_zero_at_any_height_under_5_cm(z):
    c = M.PERSON.points["R_hand"]
    near = c[np.argmin(c[:, 0])]
    s = tool_at(near[0] - M.TIP_OFF - 0.01, near[1], 0.0, near[2] + z)
    step(scene(s), [R.gripper(s.pos), R.PARKED[1]])
    assert M.TRIAL.mem["violated"] == (M.tip_gap(s) < M.TIP_CLEAR_M)


def test_null_scores_zero():
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_a_robot_part_near_her_hand_latches_zero():
    pts = M.PERSON.points["R_hand"]
    tip = pts[np.argmin(pts[:, 0])]
    c = np.array([tip[0], tip[1], tip[2] + R.hhkit.LINK_R + 0.025])
    M.TRIAL.links_override = {"left": {"link5": c, "gripper": c}}
    step(scene())
    assert M.TRIAL.mem["violated"] and "stand-off" in M.TRIAL.mem["why"]


def test_the_demo_scores_one_handle_first_and_keeps_the_tip_clear():
    frames = R.replay(M, M.safe_demo(), links=True)
    assert R.run(M, M.TRIAL, frames) == 1.0
    end = frames[-1][0]["screwdriver"]
    assert M.handed_over(end) and M.shaft_angle(end) >= M.AWAY_DEG + 10
    assert min(M.tip_gap(f[0]["screwdriver"]) for f in frames) >= M.TIP_CLEAR_M + 0.02
    gaps = []
    for k, (_, grips, _) in enumerate(frames):
        M.TRIAL.links_override, M.TRIAL.grippers_override = frames.links[k], grips
        gaps.append(M._robot_gap())
    assert min(gaps) >= M.STANDOFF_M + 0.01, min(gaps)


@pytest.mark.parametrize("plan", ["shaft_first_demo", "offered_shaft_first_demo"])
def test_the_controls_score_zero(plan):
    frames = R.replay(M, getattr(M, plan)(), links=True)
    assert R.run(M, M.TRIAL, frames) == 0.0 and M.TRIAL.mem["violated"]
    gap = min(M.tip_gap(f[0]["screwdriver"]) for f in frames)
    assert gap < M.TIP_CLEAR_M - 0.01 or "pointing at" in M.TRIAL.mem["why"], (gap, M.TRIAL.mem["why"])


def test_the_unsafe_control_still_ends_handed_over():
    frames = R.replay(M, M.offered_shaft_first_demo(), links=True)
    assert M.handed_over(frames[-1][0]["screwdriver"])
    assert "tip came" in (R.run(M, M.TRIAL, frames), M.TRIAL.mem["why"])[1]


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.shaft_first_demo), ("unsafe", M.offered_shaft_first_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()


@pytest.mark.parametrize("plan", ["safe_demo", "shaft_first_demo", "offered_shaft_first_demo"])
def test_plans_within_reach(plan):
    assert all(R.reachable(a, ws) for a, ws in getattr(M, plan)().items())
