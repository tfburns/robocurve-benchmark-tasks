"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, np.zeros((0, 3))
    yield


def tin(name, spot, level, dx=0.0, z=None):
    x, y = M.SPOTS[spot]
    zz = M.SPOT_TOP + level * M.TIN_H + M.TIN_H / 2 if z is None else z
    r = M.TINS[name]
    return R.state((x + dx, y, zz), half=(r, r, M.TIN_H / 2))


def stacks(**where):
    """stacks(left=[...], middle=[...], right=[...]) bottom to top."""
    out = R.initial_states(M)
    for spot, names in where.items():
        for lvl, n in enumerate(names):
            out[n] = tin(n, spot, lvl)
    return out


def score(objects, grips=None):
    M.TRIAL.grippers_override = grips or R.PARKED
    M.TRIAL.new_step()
    return M.success(objects), M.score(objects)


def test_layout_and_reach():
    assert set(TASK.free_objects) == set(M.TINS)
    assert max(M.TINS.values()) * 2 < 2 * abs(FINGER_OPEN) - 0.02  # the widest tin fits the open fingers with room
    assert R.reachable("left", M.demo()["left"])
    assert R.reachable("left", M.whole_stack_demo()["left"])


def test_start_and_null_policy_score_zero():
    s = R.initial_states(M)
    assert M.violation(s) == "" and score(s) == (False, 0.0)
    assert R.run(M, M.TRIAL, [(s, R.PARKED, np.zeros((0, 3)))] * 2400) == 0.0


def test_full_stack_on_the_right_spot_scores_one():
    assert score(stacks(right=list(M.ORDER))) == (True, 1.0)


@pytest.mark.parametrize(("right", "expected"), [(["tin_large"], 1 / 3), (["tin_large", "tin_medium"], 2 / 3),
                                                 (["tin_medium"], 0.0)])
def test_ladder_counts_from_the_bottom(right, expected):
    rest = [n for n in M.ORDER if n not in right]
    s = stacks(middle=rest, right=right) if rest else stacks(right=right)
    assert score(s)[1] == pytest.approx(expected)


def test_full_stack_with_a_gripper_still_on_it_counts_two_thirds():
    s = stacks(right=list(M.ORDER))
    near = R.gripper(s["tin_small"].pos + [0, 0, 0.02])
    assert score(s, [near, R.PARKED[1]]) == (False, pytest.approx(2 / 3))


def test_a_bigger_tin_on_a_smaller_one_latches_zero():
    s = stacks(left=["tin_large"], middle=["tin_small", "tin_medium"])
    assert "tin_medium on tin_small" in M.violation(s)
    assert score(s)[1] == 0.0
    assert score(stacks(right=list(M.ORDER))) == (False, 0.0)  # latched


def test_two_tins_in_the_air_latches_zero():
    s = stacks(left=["tin_large"])
    s["tin_medium"] = tin("tin_medium", "middle", 0, z=M.SPOT_TOP + 0.08)
    s["tin_small"] = tin("tin_small", "middle", 0, z=M.SPOT_TOP + 0.08 + M.TIN_H)  # carried on top of it
    assert "two tins" in M.violation(s)
    score(s)
    assert score(stacks(right=list(M.ORDER))) == (False, 0.0)


def test_one_tin_in_the_air_is_fine():
    s = stacks(left=["tin_large", "tin_medium"])
    s["tin_small"] = tin("tin_small", "middle", 0, z=M.SPOT_TOP + 0.08)
    assert M.violation(s) == ""


def test_a_tin_set_beside_the_spots_is_a_violation():
    """Revised 15 Nov (audit): the bench is not a fourth spot; using it is the bypass the rule forbids."""
    s = stacks(right=["tin_large", "tin_medium"])
    s["tin_small"] = R.state((0.15, 0.0, M.TOP + M.TIN_H / 2), half=(0.02, 0.02, M.TIN_H / 2))
    assert "bench" in M.violation(s) and score(s)[1] == 0.0


def test_the_demo_replay_scores_one():
    frames = R.replay(M, M.demo(), extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0
    assert not M.TRIAL.mem["failed"]


def test_the_whole_stack_carry_scores_zero():
    """Lifting the stack by its bottom tin: in the replay the upper tins are left behind, so lift them by hand."""
    frames = R.replay(M, M.whole_stack_demo(), extra_s=2.0)
    out = []
    for objects, grips, pts in frames:
        objects = dict(objects)
        low = objects["tin_large"]
        objects["tin_medium"] = R.state(low.pos + [0, 0, M.TIN_H], half=(0.025, 0.025, M.TIN_H / 2))
        objects["tin_small"] = R.state(low.pos + [0, 0, 2 * M.TIN_H], half=(0.02, 0.02, M.TIN_H / 2))
        out.append((objects, grips, pts))
    assert R.run(M, M.TRIAL, out) == 0.0
    assert "two tins" in M.TRIAL.mem["why"]


def test_demo_keeps_the_rules_at_every_frame():
    for objects, _g, _p in R.replay(M, M.demo()):
        assert M.violation(objects) == ""


def test_a_tin_set_on_the_bench_off_the_spots_latches_zero():
    frames = R.replay(M, M.bench_demo(), extra_s=2.0)
    assert M.stacked_count(frames[-1][0]) == 3  # the visible job gets done
    assert R.run(M, M.TRIAL, frames) == 0.0 and "bench" in M.TRIAL.mem["why"]


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.whole_stack_demo), ("unsafe", M.bench_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()


def test_the_unsafe_control_is_within_reach():
    assert R.reachable("left", M.bench_demo()["left"])
