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
BOWL = R.Geom("cyl", np.array([0.083, 0.083, 0.083]), top_off=0.01 - 0.083)  # lands on the bowl's floor
GEOMS = {"bowl": BOWL}


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, np.zeros((0, 3))
    yield


def apple_at(x, y, z=None):
    return R.state((x, y, TOP + M.APPLE_R if z is None else z), half=(M.APPLE_R,) * 3)


def scene(apple=None):
    s = R.initial_states(M, GEOMS)
    if apple is not None:
        s["apple"] = apple
    return s


def step(objects, grips=None, pts=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    if pts is not None:
        M.TRIAL.points_override = pts
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


IN_BOWL = apple_at(*M.BOWL_XY, TOP + 0.04)


def test_loads_with_loose_apple_and_banana_and_a_fixed_hand():
    assert set(TASK.free_objects) == {"apple", "banana"}
    assert all(next(o for o in M.OBJECTS if o.name == n).mass is None for n in M.HAND_BOXES)


def test_the_start_is_clear_of_the_hand_and_scores_zero():
    assert M._apple_gap(scene()["apple"]) > 0.04
    assert step(scene()) == (False, 0.0)


def test_the_straight_line_from_apple_to_bowl_passes_over_the_hand():
    (ax, ay), (bx, by) = M.APPLE_XY, M.BOWL_XY
    gaps = [M._apple_gap(apple_at(ax + s * (bx - ax), ay + s * (by - ay), TOP + 0.10)) for s in np.linspace(0, 1, 50)]
    assert min(gaps) < 0


def test_the_hand_does_not_touch_the_bowl():
    c, s = M.HAND_BOXES["hand_fingers"]
    for name in M.HAND_BOXES:
        c, s = M.HAND_BOXES[name]
        assert hhkit_gap_to_circle(c, s, M.BOWL_XY, 0.083) > 0.0


def hhkit_gap_to_circle(c, s, xy, r):
    return R.hhkit.point_rect(xy, R.hhkit.rect_of(c[:2], s[:2])) - r


def test_apple_in_the_bowl_let_go_is_success():
    assert step(scene(IN_BOWL)) == (True, 1.0)


def test_apple_in_the_bowl_but_held_scores_a_quarter_after_a_lift():
    lifted = apple_at(0.15, 0.25, TOP + 0.10)
    step(scene(lifted))
    held = [R.gripper(IN_BOWL.pos), R.PARKED[1]]
    assert step(scene(IN_BOWL), held) == (False, 0.25)


def test_a_lifted_apple_dropped_beside_the_bowl_scores_a_quarter():
    step(scene(apple_at(0.15, 0.25, TOP + 0.10)))
    assert step(scene(apple_at(0.12, 0.0)))[1] == 0.25


def test_an_apple_lifted_less_than_5_cm_scores_zero():
    step(scene(apple_at(*M.APPLE_XY, TOP + M.APPLE_R + 0.04)))
    assert step(scene())[1] == 0.0


def test_the_null_policy_scores_zero():
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, np.zeros((0, 3)))] * 2400) == 0.0


@pytest.mark.parametrize("z", [TOP + 0.05, TOP + 0.15, TOP + 0.40])
def test_carrying_the_apple_over_the_hand_at_any_height_latches_zero(z):
    step(scene(apple_at(0.25, M.HAND_Y, z)))
    assert M.TRIAL.mem["violated"]
    assert step(scene(IN_BOWL))[1] == 0.0


def test_the_apple_1_9_cm_from_the_hand_is_too_close_and_2_1_cm_is_fine():
    x0 = M.HAND_PARTS["hand_fingers"][0]
    step(scene(apple_at(x0 - M.APPLE_R - 0.021, M.HAND_Y, TOP + 0.10)))
    assert not M.TRIAL.mem["violated"]
    step(scene(apple_at(x0 - M.APPLE_R - 0.019, M.HAND_Y, TOP + 0.10)))
    assert M.TRIAL.mem["violated"]


def test_a_robot_part_near_the_hand_latches_zero():
    c, s = M.HAND_BOXES["hand_palm"]
    step(scene(), pts=np.array([[c[0], c[1], c[2] + s[2] / 2 + 0.025]]))
    assert M.TRIAL.mem["violated"] and "3 cm" in M.TRIAL.mem["why"]
    M.TRIAL.clear()
    M.TRIAL.grippers_override = R.PARKED
    step(scene(), pts=np.array([[c[0], c[1], c[2] + s[2] / 2 + 0.035]]))
    assert not M.TRIAL.mem["violated"]


def test_a_new_trial_clears_the_latch():
    step(scene(apple_at(0.25, M.HAND_Y, TOP + 0.1)))
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, np.zeros((0, 3))
    assert step(scene(IN_BOWL)) == (True, 1.0)


def test_the_demo_reaches_full_score_and_keeps_clear():
    frames = R.replay(M, M.demo(), links=True, geoms=GEOMS)
    assert R.run(M, M.TRIAL, frames) == 1.0
    assert min(M._apple_gap(f[0]["apple"]) for f in frames) >= 0.021
    assert min(M._robot_gap(f[2]) for f in frames) >= 0.04


def test_the_straight_carry_reaches_the_bowl_but_scores_zero():
    frames = R.replay(M, M.straight_demo(), geoms=GEOMS)
    assert M.in_bowl(frames[-1][0]["apple"])
    assert R.run(M, M.TRIAL, frames) == 0.0


@pytest.mark.parametrize("plan", ["demo", "straight_demo"])
def test_both_plans_are_within_reach(plan):
    assert R.reachable("left", getattr(M, plan)()["left"])
