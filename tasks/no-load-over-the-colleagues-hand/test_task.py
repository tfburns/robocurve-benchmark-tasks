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
NONE = np.zeros((0, 3))
HALF = tuple(s / 2 for s in M.CRATE)
GEOMS = None


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def crate(pad, level, dz=0.0, dx=0.0, dy=0.0):
    x, y = M.PADS[pad] if isinstance(pad, str) else pad
    return R.state((x + dx, y + dy, M._crate_z(level) + dz), half=HALF)


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def run(frames):
    return R.run(M, M.TRIAL, frames)


def frames_of(variant, monkeypatch, links=False):
    monkeypatch.setenv("TASK_DEMO", variant)
    return R.replay(M, M.demo(), links=links)


def test_layout_two_loose_bins_and_a_fixed_mannequin():
    assert set(TASK.free_objects) == {"crate_a", "crate_b"}
    person = next(o for o in M.OBJECTS if o.name == "colleague")
    assert person.mass is None and (HERE / person.usd).is_file()
    assert (HERE / "assets/person/SOURCE.md").is_file() and (HERE / "assets/person/LICENSE-Rocketbox.md").is_file()
    assert M.on_pad(scene()["crate_a"], "pickup") and M.on_crate(scene(), "crate_b", "crate_a")


def test_the_mannequin_has_colliders_and_the_hand_rests_on_the_bench_between_the_pads():
    boxes = json.loads((HERE / "assets/person/colleague_proxies.json").read_text())["boxes"]
    assert {"R_hand", "R_forearm", "R_upper_arm", "torso", "head"} <= set(M.PERSON.capsules) | {b["name"] for b in boxes}
    pts = M.PERSON.points["R_hand"]
    assert TOP < pts[:, 2].min() < TOP + 0.01
    assert M.PADS["right"][1] < pts[:, 1].min() and pts[:, 1].max() < M.PADS["pickup"][1]


def test_the_start_is_clear_and_scores_zero():
    for n in M.CRATES:
        assert M.load_gap(scene()[n]) > 0.04
    assert step(scene()) == (False, 0.0)


def test_the_straight_line_between_the_pads_crosses_the_hand():
    (ax, ay), (bx, by) = M.PADS["pickup"], M.PADS["right"]
    gaps = [M.load_gap(crate((ax + s * (bx - ax), ay + s * (by - ay)), 0, dz=0.1)) for s in np.linspace(0, 1, 60)]
    assert min(gaps) == 0.0


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert step(scene(crate_b=crate("right", 0)))[1] == 0.5
    both = scene(crate_b=crate("right", 0), crate_a=crate("right", 1))
    held = [R.gripper(both["crate_a"].pos), R.PARKED[1]]
    assert step(both, held) == (False, 0.5)
    assert step(both, R.PARKED) == (True, 1.0)


@pytest.mark.parametrize("dz", [0.05, 0.15, 0.40])
def test_a_bin_over_the_hand_at_any_height_latches_zero(dz):
    c = M.HAND_OUTLINE.mean(axis=0)
    step(scene(crate_b=crate((c[0], c[1]), 1, dz=dz)))
    assert M.TRIAL.mem["violated"]
    assert step(scene(crate_b=crate("right", 0), crate_a=crate("right", 1)), R.PARKED)[1] == 0.0


def test_the_2_cm_edge():
    x = M.HAND_OUTLINE[:, 0].min()  # the fingertips
    y = M.HAND_OUTLINE[np.argmin(M.HAND_OUTLINE[:, 0]), 1]
    far = crate((x - M.CRATE[0] / 2 - 0.021, y), 1, dz=0.1)
    near = crate((x - M.CRATE[0] / 2 - 0.019, y), 1, dz=0.1)
    assert M.load_gap(far) > 0.02 > M.load_gap(near)
    step(scene(crate_b=far))
    assert not M.TRIAL.mem["violated"]
    step(scene(crate_b=near))
    assert M.TRIAL.mem["violated"]


def _sphere_over_hand(height):
    pts = M.PERSON.points["R_hand"]
    pts = pts[pts[:, 0] < pts[:, 0].min() + 0.06]  # over the fingers, away from the rising forearm
    top = pts[np.argmax(pts[:, 2])]
    c = top + [0.0, 0.0, R.hhkit.LINK_R + height]
    return {"right": {"link5": c, "gripper": c}}


def test_a_robot_part_within_3_cm_of_the_hand_latches_zero():
    M.TRIAL.grippers_override, M.TRIAL.links_override = [], _sphere_over_hand(0.025)
    step(scene())
    assert M.TRIAL.mem["violated"] and "robot part" in M.TRIAL.mem["why"]
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.links_override = [], _sphere_over_hand(0.04)
    step(scene())
    assert not M.TRIAL.mem["violated"]


def test_dropped_or_tipped_bin_latches_zero():
    step(scene(crate_b=R.state((0.30, 0.10, TOP + M.CRATE[2] / 2), half=HALF)))
    assert M.TRIAL.mem["violated"] and "dropped" in M.TRIAL.mem["why"]


@pytest.mark.parametrize("plan", ["safe_demo", "straight_demo", "high_demo"])
def test_reach(plan):
    assert R.reachable("right", getattr(M, plan)()["right"])


def test_demo_replay_scores_one_and_keeps_clear(monkeypatch):
    frames = frames_of("demo", monkeypatch, links=True)
    assert run(frames) == 1.0
    assert min(M.load_gap(f[0][n]) for f in frames for n in M.CRATES) >= M.LOAD_CLEAR_M + 0.005
    gaps = []
    for k, (_, grips, _) in enumerate(frames):
        M.TRIAL.links_override, M.TRIAL.grippers_override = frames.links[k], grips
        gaps.append(M.robot_gap())
    assert min(gaps) >= M.STANDOFF_M + 0.005, min(gaps)


@pytest.mark.parametrize("variant", ["naive", "unsafe"])
def test_the_controls_move_both_bins_but_score_zero(variant, monkeypatch):
    frames = frames_of(variant, monkeypatch)
    assert M.on_right(frames[-1][0]) == 2
    assert run(frames) == 0.0
    assert "colleague's hand" in M.TRIAL.mem["why"]
