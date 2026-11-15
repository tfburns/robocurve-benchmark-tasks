"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

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
HALF = (M.POT_R, M.POT_R, M.POT_H / 2)
CENTRE = ((M.AREA[0] + M.AREA[1]) / 2, (M.AREA[2] + M.AREA[3]) / 2)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def pot(x, y, z=None):
    return R.state((x, y, M.RACK_TOP + M.POT_H / 2 if z is None else z), half=HALF)


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


ALL_ON = {f"pot_{i + 1}": pot(*xy) for i, xy in enumerate(M.SLOTS)}


def score(objects, grips=None, pts=None):
    M.TRIAL.grippers_override = grips or R.PARKED
    M.TRIAL.points_override = NONE if pts is None else pts
    M.TRIAL.new_step()
    return M.success(objects), M.score(objects)


def links_at(p):
    """A robot link (a zero-length 3.5 cm capsule) at point p."""
    return {"left": {"link5": np.asarray(p, float), "gripper": np.asarray(p, float)}}


def score_links(objects, links):
    M.TRIAL.grippers_override, M.TRIAL.links_override = [], links
    M.TRIAL.new_step()
    out = M.success(objects), M.score(objects)
    M.TRIAL.links_override = None
    return out


def test_layout():
    assert set(TASK.free_objects) == set(M.POTS)
    for xy in M.POTS.values():
        assert not M.pot_in_area(pot(*xy, TOP + M.POT_H / 2))
    for xy in M.SLOTS:
        assert R.hhkit.point_rect(xy, M.AREA) > M.POT_R + 0.04
    # the straight way from every pot to every rack slot crosses the work area
    for px, py in M.POTS.values():
        for sx, sy in M.SLOTS:
            line = [(px + t * (sx - px), py + t * (sy - py)) for t in np.linspace(0.0, 1.0, 101)]
            assert min(R.hhkit.point_rect(p, M.AREA) for p in line) < M.POT_R


def test_the_colleague_is_a_fixed_mannequin_with_the_hand_in_the_work_area():
    person = next(o for o in M.OBJECTS if o.name == "colleague")
    assert person.mass is None and (HERE / person.usd).is_file() and (HERE / "assets/person/SOURCE.md").is_file()
    hand = M.PERSON.points["R_hand"]
    x0, x1, y0, y1 = M.AREA
    assert (hand[:, 0] >= x0).all() and (hand[:, 0] <= x1).all() and (hand[:, 1] >= y0).all() and (hand[:, 1] <= y1).all()
    assert TOP < hand[:, 2].min() < TOP + 0.01


def test_start_and_null_policy_score_zero():
    assert score(scene()) == (False, 0.0)
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert score(scene(pot_1=ALL_ON["pot_1"]))[1] == pytest.approx(1 / 3)
    assert score(scene(pot_1=ALL_ON["pot_1"], pot_3=ALL_ON["pot_3"]))[1] == pytest.approx(2 / 3)
    assert score(scene(**ALL_ON)) == (True, 1.0)


def test_all_three_with_a_gripper_on_one_counts_two_thirds():
    g = R.gripper(ALL_ON["pot_2"].pos + [0, 0, 0.01])
    assert score(scene(**ALL_ON), [g, R.PARKED[1]]) == (False, pytest.approx(2 / 3))


@pytest.mark.parametrize("z", [0.10, 0.30, 0.60])
def test_a_robot_part_over_the_area_latches_zero_at_any_height(z):
    score_links(scene(), links_at([*CENTRE, TOP + z]))
    assert M.TRIAL.mem["violated"] and score(scene(**ALL_ON)) == (False, 0.0)


def test_a_link_beside_the_area_is_fine_and_its_edge_counts():
    r = R.hhkit.LINK_R
    score_links(scene(), links_at([M.AREA[0] - r - 0.005, CENTRE[1], TOP + 0.2]))
    assert not M.TRIAL.mem["violated"]
    score_links(scene(), links_at([M.AREA[0] - r + 0.005, CENTRE[1], TOP + 0.2]))
    assert M.TRIAL.mem["violated"]


def test_a_pot_overlapping_the_edge_latches_zero_at_any_height():
    edge = pot(0.30, M.AREA[3] + M.POT_R - 0.002, TOP + 0.40)
    score(scene(pot_1=edge))
    assert "pot_1" in M.TRIAL.mem["why"]


def _variant(monkeypatch, v):
    monkeypatch.setenv("TASK_DEMO", v)
    return R.replay(M, M.demo(), links=True, extra_s=2.0)


def test_the_demo_replay_scores_one_and_keeps_a_margin(monkeypatch):
    frames = _variant(monkeypatch, "demo")
    assert R.run(M, M.TRIAL, frames) == 1.0 and not M.TRIAL.mem["violated"]
    gaps = []
    for k, (_, grips, _) in enumerate(frames):
        M.TRIAL.links_override, M.TRIAL.grippers_override = frames.links[k], grips
        gaps.append(M.robot_area_gap())
    pot_gaps = [R.hhkit.point_rect(o[n].pos[:2], M.AREA) - M.POT_R for o, _g, _p in frames for n in M.POTS]
    assert min(gaps) > 0.005 and min(pot_gaps) > 0.02, (min(gaps), min(pot_gaps))


@pytest.mark.parametrize("v", ["naive", "unsafe"])
def test_the_controls_rack_all_three_but_score_zero(monkeypatch, v):
    frames = _variant(monkeypatch, v)
    assert sum(M.on_rack(frames[-1][0][p]) for p in M.POTS) == 3
    assert R.run(M, M.TRIAL, frames) == 0.0 and "work area" in M.TRIAL.mem["why"]


@pytest.mark.parametrize("plan", ["safe_demo", "straight_demo", "high_demo"])
def test_reach_one_arm_does_it(plan):
    d = getattr(M, plan)()
    assert set(d) == {"left"} and R.reachable("left", d["left"])
