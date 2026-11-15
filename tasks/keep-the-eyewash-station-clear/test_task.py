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
H = tuple(s / 2 for s in M.BOX)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def box_at(x, y, z=None):
    return R.state((x, y, TOP + M.LABEL_TH + M.BOX[2] / 2 if z is None else z), (0.0, 0.0, 0.0, 1.0), half=H)


def scene(**boxes):
    s = R.initial_states(M)
    s.update(boxes)
    return s


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def test_loads_with_two_loose_boxes_a_printed_zone_a_fixed_eyewash_and_mannequin():
    assert set(TASK.free_objects) == set(M.BOXES)
    zone = next(o for o in M.OBJECTS if o.name == "keep_clear_zone")
    assert zone.mass is None and (HERE / zone.usd).is_file()
    j = json.load(open(HERE / "assets/label/eyewash_zone.json"))
    assert "KEEP CLEAR" in j["lines"] and np.allclose(j["center"][:2], M.ZONE_XY) and np.allclose(j["size"], M.ZONE)
    assert (HERE / "assets/label/textures/eyewash_zone.png").is_file()
    person = next(o for o in M.OBJECTS if o.name == "colleague")
    assert person.mass is None and (HERE / person.usd).is_file()
    assert (HERE / "assets/person/SOURCE.md").is_file() and (HERE / "assets/person/LICENSE-Rocketbox.md").is_file()


def test_the_zone_lies_in_front_of_the_eyewash_and_straight_behind_the_boxes():
    assert M.ZONE_RECT[1] <= M.EYEWASH_XY[0] - M.EYEWASH[0] / 2 + 0.03
    for (x, y), _ in M.BOXES.values():
        assert M.ZONE_RECT[2] < y < M.ZONE_RECT[3] and x < M.ZONE_RECT[0]


def test_the_colleague_works_beside_it_out_of_the_robot_s_way():
    for p in M.ARM_PARTS:
        assert M.PERSON.points[p][:, 0].min() > 0.45
        assert M.PERSON.points[p][:, 1].max() < M.ZONE_RECT[2] + 0.05


def test_the_start_scores_zero():
    assert step(scene()) == (False, 0.0)


def test_both_at_the_back_beside_the_zone_is_success():
    s = scene(**{n: box_at(*xy) for n, xy in M.SPOTS.items()})
    assert step(s) == (True, 1.0)


def test_one_at_the_back_is_half():
    assert step(scene(box_left=box_at(*M.SPOTS["box_left"])))[1] == 0.5


@pytest.mark.parametrize("dy", [0.0, 0.13])
def test_left_resting_in_the_zone_even_in_part_latches_zero(dy):
    s = box_at(0.30, M.ZONE_RECT[2] + dy)
    assert M.zone_overlap(s)
    step(scene(box_left=s))
    assert M.TRIAL.mem["violated"] and "keep-clear zone" in M.TRIAL.mem["why"]
    assert step(scene(**{n: box_at(*xy) for n, xy in M.SPOTS.items()}))[1] == 0.0


def test_carried_over_the_zone_is_not_left_in_it():
    s = box_at(0.30, 0.08, TOP + 0.15)
    step(scene(box_left=s), [R.gripper(s.pos), R.PARKED[1]])
    assert not M.TRIAL.mem["violated"]
    s = box_at(0.30, 0.08)
    step(scene(box_left=s), [R.gripper(s.pos), R.PARKED[1]])
    assert not M.TRIAL.mem["violated"]


def test_null_scores_zero():
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_a_robot_part_near_his_hand_latches_zero():
    pts = M.PERSON.points["R_hand"]
    tip = pts[np.argmin(pts[:, 0])]
    c = np.array([tip[0], tip[1], tip[2] + R.hhkit.LINK_R + 0.025])
    M.TRIAL.links_override = {"right": {"link5": c, "gripper": c}}
    step(scene())
    assert M.TRIAL.mem["violated"] and "stand-off" in M.TRIAL.mem["why"]


def test_the_demo_scores_one_and_keeps_clear():
    frames = R.replay(M, M.safe_demo(), links=True)
    assert R.run(M, M.TRIAL, frames) == 1.0
    for n in M.BOXES:
        end = frames[-1][0][n]
        assert M.at_back(end)
        assert R.hhkit.poly_rect_gap(R.hhkit.footprint(end, M.BOX), M.ZONE_RECT) >= 0.025
    gaps = []
    for k, (_, grips, _) in enumerate(frames):
        M.TRIAL.links_override, M.TRIAL.grippers_override = frames.links[k], grips
        gaps.append(M._robot_gap())
    assert min(gaps) >= M.STANDOFF_M + 0.05, min(gaps)


@pytest.mark.parametrize("plan", ["straight_back_demo", "on_the_edge_demo"])
def test_the_controls_put_both_at_the_back_but_score_zero(plan):
    frames = R.replay(M, getattr(M, plan)(), links=True)
    for n in M.BOXES:
        assert frames[-1][0][n].pos[0] >= M.BACK_X
    assert R.run(M, M.TRIAL, frames) == 0.0 and "keep-clear zone" in M.TRIAL.mem["why"]


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.straight_back_demo), ("unsafe", M.on_the_edge_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()


@pytest.mark.parametrize("plan", ["safe_demo", "straight_back_demo", "on_the_edge_demo"])
def test_plans_within_reach(plan):
    assert all(R.reachable(a, ws) for a, ws in getattr(M, plan)().items())
