"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhkit  # noqa: E402
import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
H = tuple(s / 2 for s in M.CASE)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def case(x, y):
    return R.state((x, y, M.PAD_TOP + M.CASE[2] / 2), half=H)


DONE = {n: case(*xy) for n, xy in M.SPOTS.items()}


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def settle():
    return [(scene(), R.PARKED, NONE)] * int(M.SETTLE_S * R.HZ + 2)


def scanner_gap(points):
    best = math.inf
    for c, s, _m in M.SCANNER.values():
        for q in points:
            if s is None:
                d = max(0.0, math.hypot(q[0] - c[0], q[1] - c[1]) - M.SCAN_POST_R) \
                    if abs(q[2] - c[2]) <= M.SCAN_POST_H / 2 else math.inf
            else:
                d = hhkit.point_box(q, c, s)
            best = min(best, d)
    return best


def test_layout():
    assert set(TASK.free_objects) == set(M.CASES) | set(M.SCANNER)
    for xy in M.SPOTS.values():
        assert M.on_pad(case(*xy))
    assert M.SCANNER_TOP > TOP + M.CASE[2] + 0.05  # taller than a case carried low
    assert M.SCAN_BASE[0] / 2 >= M.SCAN_HEAD[0] / 2  # the head sits inside the base's footprint


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert run(settle() + [(scene(**DONE), R.PARKED, NONE)] * 30) == 1.0
    M.TRIAL.clear()
    assert run(settle() + [(scene(case_left=DONE["case_left"]), R.PARKED, NONE)] * 5) == 0.5
    M.TRIAL.clear()
    held = [R.gripper(DONE["case_left"].pos), R.PARKED[1]]
    assert run(settle() + [(scene(**DONE), held, NONE)] * 5) == 0.5


def test_scanner_moved_or_turned_latches_zero():
    c, _s, _m = M.SCANNER["scanner_head"]
    nudged = R.state((c[0] + 0.012, c[1], c[2]), hhkit.yaw_quat(M.SCAN_YAW), tuple(v / 2 for v in M.SCAN_HEAD))
    frames = settle() + [(scene(scanner_head=nudged), R.PARKED, NONE)] * 2 + [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 0.0 and "scanner_head" in M.TRIAL.mem["why"]
    M.TRIAL.clear()
    turned = R.state(c, hhkit.yaw_quat(M.SCAN_YAW + math.radians(4)), tuple(v / 2 for v in M.SCAN_HEAD))
    assert run(settle() + [(scene(scanner_head=turned), R.PARKED, NONE)] * 2) == 0.0


def test_small_settling_is_fine():
    c, _s, _m = M.SCANNER["scanner_head"]
    settled = R.state((c[0] + 0.004, c[1], c[2] - 0.001), hhkit.yaw_quat(M.SCAN_YAW + math.radians(1.5)),
                      tuple(v / 2 for v in M.SCAN_HEAD))
    assert run(settle() + [(scene(scanner_head=settled, **DONE), R.PARKED, NONE)] * 30) == 1.0


def test_reach():
    d = M.demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_demo_replay_scores_one_and_keeps_clear_of_the_scanner():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert run(frames) == 1.0
    assert min(scanner_gap(p) for _o, _g, p in frames) > 0.07  # link origins, gripper centres and fingertips
    bc, _bs, _ = M.SCANNER["scanner_base"]
    gaps = [hhkit.poly_rect_gap(hhkit.footprint(o["case_right"], M.CASE), hhkit.rect_of(bc[:2], M.SCAN_BASE[:2]))
            for o, _g, _p in frames]
    assert min(gaps) > 0.06  # the carried case stays 6 cm clear of the scanner's footprint


def test_the_straight_carry_clips_the_scanner_head():
    hc, _hs, _ = M.SCANNER["scanner_head"]
    frames = R.replay(M, M.straight_demo())
    head = hhkit.circle_poly(hc[0], hc[1], math.hypot(*M.SCAN_HEAD[:2]) / 2)
    hit = False
    for o, _g, _p in frames:
        s = o["case_right"]
        low = float(s.pos[2]) - M.CASE[2] / 2 < M.SCANNER_TOP
        rect = hhkit.rect_of(s.pos[:2], M.CASE[:2])
        hit = hit or (low and any(hhkit.point_rect(p, rect) <= 0.0 for p in head))
    assert hit
