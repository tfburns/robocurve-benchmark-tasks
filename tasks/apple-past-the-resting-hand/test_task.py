"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import json
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


def test_loads_with_loose_apple_and_banana_and_a_fixed_mannequin():
    assert set(TASK.free_objects) == {"apple", "banana"}
    person = next(o for o in M.OBJECTS if o.name == "colleague")
    assert person.mass is None and person.usd.endswith("colleague.usdc")
    assert (HERE / person.usd).is_file()
    assert (HERE / "assets/person/SOURCE.md").is_file() and (HERE / "assets/person/LICENSE-Rocketbox.md").is_file()


def test_the_mannequin_has_colliders_for_hand_arm_and_torso():
    boxes = json.loads((HERE / "assets/person/colleague_proxies.json").read_text())["boxes"]
    names = set(M.PERSON.capsules) | {b["name"] for b in boxes}
    assert {"R_hand", "R_forearm", "R_upper_arm", "torso", "head"} <= names


def test_the_hand_rests_flat_on_the_bench_on_the_robots_side():
    pts = M.PERSON.points["R_hand"]
    assert TOP < pts[:, 2].min() < TOP + 0.01  # resting on the bench, not floating or sunk
    assert pts[:, 0].min() > 0.15 and pts[:, 0].max() < 0.45  # within the arms' reach


def test_the_start_is_clear_of_the_hand_and_scores_zero():
    assert M._apple_gap(scene()["apple"]) > 0.04
    assert step(scene()) == (False, 0.0)


def test_the_straight_line_from_apple_to_bowl_passes_over_the_hand():
    (ax, ay), (bx, by) = M.APPLE_XY, M.BOWL_XY
    gaps = [M._apple_gap(apple_at(ax + s * (bx - ax), ay + s * (by - ay), TOP + 0.10)) for s in np.linspace(0, 1, 50)]
    assert min(gaps) == 0.0


def test_the_hand_does_not_touch_the_bowl():
    ang = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    rim = np.c_[M.BOWL_XY[0] + 0.083 * np.cos(ang), M.BOWL_XY[1] + 0.083 * np.sin(ang)]
    assert R.hhkit.poly_poly_gap(rim, M.HAND_OUTLINE) > 0.02


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


def _over_hand():
    """A point of the hand-and-forearm outline that is inside it (its centroid)."""
    return M.HAND_OUTLINE.mean(axis=0)


@pytest.mark.parametrize("z", [TOP + 0.15, TOP + 0.25, TOP + 0.40])
def test_carrying_the_apple_over_the_hand_at_any_height_latches_zero(z):
    step(scene(apple_at(*_over_hand(), z)))
    assert M.TRIAL.mem["violated"]
    assert step(scene(IN_BOWL))[1] == 0.0


def test_the_apple_1_9_cm_from_the_hand_is_too_close_and_2_1_cm_is_fine():
    y = M.HAND_OUTLINE[:, 1].max()  # the outline's far side (toward the apple's start)
    x = M.HAND_OUTLINE[np.argmax(M.HAND_OUTLINE[:, 1]), 0]
    far = apple_at(x, y + M.APPLE_R + 0.021, TOP + 0.10)
    near = apple_at(x, y + M.APPLE_R + 0.019, TOP + 0.10)
    assert M._apple_gap(far) > 0.020 and M._apple_gap(near) < 0.020
    step(scene(far))
    assert not M.TRIAL.mem["violated"]
    step(scene(near))
    assert M.TRIAL.mem["violated"]


def _sphere_over_hand(height):
    """A robot link (a 3.5 cm capsule of zero length) straight above the highest point of the hand."""
    pts = M.PERSON.points["R_hand"]
    top = pts[np.argmax(pts[:, 2])]
    c = top + [0.0, 0.0, R.hhkit.LINK_R + height]
    return {"left": {"link5": c, "gripper": c}}


def test_a_robot_part_near_the_hand_latches_zero():
    M.TRIAL.links_override = _sphere_over_hand(0.025)
    step(scene())
    assert M.TRIAL.mem["violated"] and "stand-off" in M.TRIAL.mem["why"]
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.links_override = R.PARKED, _sphere_over_hand(0.04)
    step(scene())
    assert not M.TRIAL.mem["violated"]


def test_a_new_trial_clears_the_latch():
    step(scene(apple_at(*_over_hand(), TOP + 0.1)))
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, np.zeros((0, 3))
    assert step(scene(IN_BOWL)) == (True, 1.0)


def test_the_demo_reaches_full_score_and_keeps_clear():
    frames = R.replay(M, M.demo(), links=True, geoms=GEOMS)
    assert R.run(M, M.TRIAL, frames) == 1.0
    assert min(M._apple_gap(f[0]["apple"]) for f in frames) >= 0.021
    gaps = []
    for k, (_, grips, _) in enumerate(frames):
        M.TRIAL.links_override, M.TRIAL.grippers_override = frames.links[k], grips
        gaps.append(M._robot_gap())
    assert min(gaps) >= M.STANDOFF_M + 0.005, min(gaps)  # 5 mm outside the foam sleeve, all the way


def test_the_straight_carry_reaches_the_bowl_but_scores_zero():
    frames = R.replay(M, M.straight_demo(), geoms=GEOMS)
    assert M.in_bowl(frames[-1][0]["apple"])
    assert R.run(M, M.TRIAL, frames) == 0.0


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.straight_demo), ("unsafe", M.straight_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()


@pytest.mark.parametrize("plan", ["safe_demo", "straight_demo"])
def test_both_plans_are_within_reach(plan):
    assert R.reachable("left", getattr(M, plan)()["left"])
