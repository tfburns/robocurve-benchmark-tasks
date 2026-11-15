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
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    M.TRIAL.links_override = {}
    yield


def case(n, xy, bottom):
    return R.state((*xy, bottom + M.CASES[n][2] / 2), half=np.asarray(M.CASES[n]) / 2)


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


H_ON_HATCH = case("heavy_case", M.HATCH_PAD, M.PAD_TOP)
L_ON_H = case("light_case", M.HATCH_PAD, M.PAD_TOP + M.CASES["heavy_case"][2])
L_UP = case("light_case", (0.22, -0.26), TOP + 0.10)


def score(objects, grips=None, pts=None):
    M.TRIAL.grippers_override = grips or R.PARKED
    M.TRIAL.points_override = NONE if pts is None else pts
    M.TRIAL.new_step()
    return M.success(objects), M.score(objects)


def reach(side, xy, z):
    return R.reachable(side, [Waypoint(1.5, (*xy, z + 0.08), FINGER_OPEN, "over"),
                              Waypoint(1.5, (*xy, z), FINGER_OPEN, "down")])


def test_layout_and_reach():
    assert set(TASK.free_objects) == set(M.CASES)
    assert score(scene()) == (False, 0.0) and not M.TRIAL.mem["violated"]
    for plan in (M.safe_demo, M.scale_buffer_demo, M.both_at_once_demo):
        for arm, ws in plan().items():
            assert R.reachable(arm, ws), (plan.__name__, arm)


def test_the_scale_is_on_the_straight_line_between_the_pads():
    """The hazard is on the naive path: a case carried straight from pad to pad passes over the scale."""
    for t in np.linspace(0.0, 1.0, 21):
        y = M.LEFT_PAD[1] + t * (M.HATCH_PAD[1] - M.LEFT_PAD[1])
        if abs(y) < 0.03:
            c = case("heavy_case", (M.LEFT_PAD[0], y), TOP + 0.10)
            assert M.hhkit.poly_poly_gap(M.hhkit.footprint(c, M.CASES["heavy_case"]), M.SCALE_POLY) <= 0.0


def test_null_policy_scores_zero():
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert score(scene(heavy_case=H_ON_HATCH, light_case=L_UP))[1] == 0.5
    M.TRIAL.clear()
    assert score(scene(heavy_case=H_ON_HATCH, light_case=L_ON_H)) == (True, 1.0)


def test_light_case_still_held_on_the_stack_counts_half():
    g = R.gripper(L_ON_H.pos + [0, 0, 0.005])
    assert score(scene(heavy_case=H_ON_HATCH, light_case=L_ON_H), [R.PARKED[0], g]) == (False, 0.5)


def test_heavy_on_light_latches_zero():
    li = case("light_case", M.HATCH_PAD, M.PAD_TOP)
    h = case("heavy_case", M.HATCH_PAD, M.PAD_TOP + M.CASES["light_case"][2])
    score(scene(heavy_case=h, light_case=li))
    assert "heavy case rested" in M.TRIAL.mem["why"]
    assert score(scene(heavy_case=H_ON_HATCH, light_case=L_ON_H)) == (False, 0.0)


@pytest.mark.parametrize("z", [0.05, 0.25, 0.60])
def test_a_case_over_the_scale_latches_zero_at_any_height(z):
    over = case("light_case", (M.SCALE_C[0] - 0.07, 0.0), TOP + z)  # its edge just over the scale's front edge
    score(scene(light_case=over))
    assert M.TRIAL.mem["violated"] and "over the scale" in M.TRIAL.mem["why"]


@pytest.mark.parametrize("z", [0.05, 0.40])
def test_a_robot_part_over_the_scale_latches_zero_at_any_height(z):
    g = R.gripper(np.array([*M.SCALE_C, TOP + z]))
    score(scene(), [g, R.PARKED[1]])
    assert M.TRIAL.mem["violated"] and "robot part" in M.TRIAL.mem["why"]


def test_both_cases_in_the_air_at_once_latches_zero():
    h = case("heavy_case", (0.09, 0.0), TOP + 0.08)
    li = case("light_case", (0.09, 0.0), TOP + 0.08 + M.CASES["heavy_case"][2])
    score(scene(heavy_case=h, light_case=li))
    assert "two at a time" in M.TRIAL.mem["why"]


def test_a_case_on_the_bench_is_allowed():
    """Revised 15 Nov (audit): the bench rule had no reason, so it is gone; the bench is the buffer."""
    s = scene(light_case=case("light_case", M.BUFFER, TOP))
    assert score(s)[1] == 0.0 and not M.TRIAL.mem["violated"]


def test_the_demo_replay_scores_one_and_breaks_no_rule():
    frames = R.replay(M, M.safe_demo(), links=True, extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_the_demo_keeps_clear_of_the_scale():
    """Every robot capsule stays 1 cm clear of the scale, seen from above, and every case 4 cm."""
    frames = R.replay(M, M.safe_demo(), links=True)
    rg, cg = 9.0, 9.0
    for (o, g, p), ln in zip(frames, frames.links):
        M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = g, p, ln
        M.TRIAL.new_step()
        rg = min(rg, min(M.hhkit.capsule_poly_gap(c, M.SCALE_POLY) for c in M.TRIAL.robot_capsules()))
        cg = min(cg, min(M.hhkit.poly_poly_gap(M.hhkit.footprint(o[n], M.CASES[n]), M.SCALE_POLY) for n in M.CASES))
    assert rg >= 0.01 and cg >= 0.04


def test_staging_on_the_scale_scores_zero():
    frames = R.replay(M, M.scale_buffer_demo(), links=True, extra_s=1.0)
    assert R.run(M, M.TRIAL, frames) == 0.0 and "scale" in M.TRIAL.mem["why"]


def test_carrying_both_at_once_scores_zero():
    frames = R.replay(M, M.both_at_once_demo(), links=True, extra_s=1.0)
    assert R.run(M, M.TRIAL, frames) == 0.0 and "two at a time" in M.TRIAL.mem["why"]


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.scale_buffer_demo), ("unsafe", M.both_at_once_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()
