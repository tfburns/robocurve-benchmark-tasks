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
    d = M.demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_both_arms_reach_both_pads():
    """Both arms can reach both pads (they're in front), which is what lets one arm hold while the other carries."""
    for side in ("left", "right"):
        assert reach(side, M.LEFT_PAD, M.PAD_TOP + 0.03) and reach(side, M.HATCH_PAD, M.PAD_TOP + 0.03)


def test_null_policy_scores_zero():
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert score(scene(heavy_case=H_ON_HATCH, light_case=L_UP))[1] == 0.5
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


@pytest.mark.parametrize("z", [0.05, 0.25])
def test_a_case_over_the_scale_latches_zero(z):
    over = case("light_case", (M.SCALE_C[0] - 0.06, 0.0), TOP + z)  # its edge just over the scale's edge
    score(scene(light_case=over))
    assert M.TRIAL.mem["violated"] and "over the scale" in M.TRIAL.mem["why"]


def test_a_robot_part_over_the_scale_latches_zero():
    score(scene(), pts=np.array([[*M.SCALE_C, TOP + 0.28]]))
    assert M.TRIAL.mem["violated"]


def test_a_case_set_on_the_bench_latches_zero():
    score(scene(light_case=case("light_case", (0.12, 0.0), TOP)))
    assert "off the pads" in M.TRIAL.mem["why"]


def test_the_demo_replay_scores_one_and_breaks_no_rule():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_the_demo_keeps_clear_of_the_scale():
    frames = R.replay(M, M.demo(), links=True)
    gaps = [R.hhkit.point_rect(p[:2], M.scale_rect()) for _o, _g, pts in frames for p in pts if p[2] <= TOP + 0.30]
    assert min(gaps) > 0.02


def test_staging_on_the_scale_scores_zero():
    frames = R.replay(M, M.scale_buffer_demo(), links=True, extra_s=1.0)
    assert R.run(M, M.TRIAL, frames) == 0.0 and "scale" in M.TRIAL.mem["why"]
