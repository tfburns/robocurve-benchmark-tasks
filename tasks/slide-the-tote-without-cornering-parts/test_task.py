"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
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
HALF_PART = (M.PART / 2,) * 3


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, np.zeros((0, 3))
    yield


def part(x, y, yaw=0.0, z=None):
    return R.state((x, y, TOP + M.PART / 2 if z is None else z), R.hhkit.yaw_quat(math.radians(yaw)), HALF_PART)


def tote(x, y):
    return R.state((x, y, TOP + M.TOTE[2] / 2), half=np.asarray(M.TOTE) / 2)


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def score(objects):
    M.TRIAL.new_step()
    return M.success(objects), M.score(objects)


IN_BAY = tote(*M.BAY_C)


def test_layout():
    assert set(TASK.free_objects) == {"tote", *M.PARTS}
    assert M.TOTE[0] > 0.094  # wider than the gripper's 9.4 cm opening: it has to be pushed
    s = scene()
    assert M.against_wall(s["parts_box_a"]) and not M.against_wall(s["parts_box_b"])
    assert M.tote_progress(s["tote"]) == 0.0


def test_the_straight_push_path_runs_through_the_parts_box_in_the_bay():
    a = M.PART_XY["parts_box_a"]
    assert M.BAY[0] <= a[0] <= M.BAY[1] and M.BAY[2] <= a[1] <= M.BAY[3]
    assert abs(a[1] - M.TOTE_START[1]) < (M.TOTE[1] + M.PART) / 2


def test_the_start_and_the_null_policy_score_zero():
    assert score(scene()) == (False, 0.0)
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, np.zeros((0, 3)))] * 2400) == 0.0


def test_tote_in_bay_with_parts_clear_is_full_score():
    assert score(scene(tote=IN_BAY, parts_box_a=part(*M.CLEAR_XY))) == (True, 1.0)


@pytest.mark.parametrize(("frac", "expected"), [(0.2, 0.0), (0.3, 0.25), (0.55, 0.5), (0.8, 0.75), (0.85, 0.75)])
def test_progress_ladder(frac, expected):
    (sx, sy), (bx, by) = M.TOTE_START, M.BAY_C
    t = tote(sx + frac * (bx - sx), sy + frac * (by - sy))
    assert score(scene(tote=t, parts_box_a=part(*M.CLEAR_XY)))[1] == pytest.approx(expected)


@pytest.mark.parametrize("side", ["right", "left"])
def test_a_parts_box_jammed_in_a_corner_scores_zero(side):
    y = M.Y0 + M.PART / 2 + 0.005 if side == "right" else M.Y1 - M.PART / 2 - 0.005
    jammed = part(M.X1 - M.PART / 2 - 0.005, y)
    assert M.cornered(jammed)
    assert score(scene(tote=IN_BAY if side == "left" else tote(0.30, -0.08), parts_box_b=jammed))[1] == 0.0


def test_a_turned_box_is_measured_by_its_corners():
    turned = part(M.X1 - M.PART / math.sqrt(2) - 0.009, M.Y1 - M.PART / math.sqrt(2) - 0.009, yaw=45)
    assert M.cornered(turned)


def test_a_parts_box_flush_against_one_wall_caps_at_half():
    flush = part(M.X1 - M.PART / 2 - 0.005, 0.05)
    assert M.against_wall(flush) and not M.cornered(flush)
    assert score(scene(tote=IN_BAY, parts_box_a=flush)) == (False, 0.5)


def test_a_parts_box_knocked_off_the_bench_caps_at_half():
    assert score(scene(tote=IN_BAY, parts_box_a=part(0.05, 0.0)))[1] == 0.5


def test_a_tipped_tote_makes_no_progress():
    q = (math.sin(math.pi / 8), 0, 0, math.cos(math.pi / 8))
    tipped = R.state((*M.BAY_C, TOP + 0.05), q, np.asarray(M.TOTE) / 2)
    assert score(scene(tote=tipped, parts_box_a=part(*M.CLEAR_XY)))[1] == 0.0


def push_replay(plan):
    """Kinematic replay plus a simple push: a closed gripper at tote height behind the tote drives it forward (+x),
    and the tote drives a parts box in its way forward until the box meets the far wall, where the tote stops."""
    frames = R.replay(M, plan)
    tx, ty = M.TOTE_START
    own, last_replay, out = {}, {}, []
    for objects, grips, pts in frames:
        tcp = grips[1]["tcp"]
        if abs(tcp[1] - ty) < M.TOTE[1] / 2 and tcp[2] < TOP + M.TOTE[2] and tcp[0] < tx:
            tx = max(tx, tcp[0] + M.TOTE[0] / 2)
        for n in M.PARTS:
            rp = objects[n].pos.copy()
            if n not in own or not np.allclose(rp, last_replay[n]):
                own[n] = rp.copy()  # the replay carried or set it down
            last_replay[n] = rp
            p = own[n]
            front = tx + M.TOTE[0] / 2
            if abs(p[1] - ty) < (M.TOTE[1] + M.PART) / 2 and p[0] > tx and p[0] - M.PART / 2 < front:
                p[0] = min(front + M.PART / 2, M.X1 - M.PART / 2)
                tx = min(tx, p[0] - M.PART / 2 - M.TOTE[0] / 2)
            objects[n] = R.state(p, half=HALF_PART)
        objects["tote"] = tote(tx, ty)
        out.append((objects, grips, pts))
    return out


def test_the_demo_reaches_full_score():
    assert R.run(M, M.TRIAL, push_replay(M.demo())) == 1.0


def test_pushing_straight_in_jams_the_parts_box_in_the_corner_and_scores_zero():
    frames = push_replay(M.straight_push_demo())
    assert M.cornered(frames[-1][0]["parts_box_a"])
    assert R.run(M, M.TRIAL, frames) == 0.0


def test_the_demo_is_within_reach():
    assert R.reachable("right", M.demo()["right"])
