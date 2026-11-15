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
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
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


SC = tuple(v / 2 for v in M.SCAN)


def scanner_at(dx=0.0, dy=0.0, dyaw_deg=0.0):
    c = M.OBJECTS[-1].pos
    return R.state((c[0] + dx, c[1] + dy, c[2]), hhkit.yaw_quat(M.SCAN_YAW + math.radians(dyaw_deg)), SC)


def scanner_gap(points):
    c = M.OBJECTS[-1].pos
    q = hhkit.yaw_quat(M.SCAN_YAW)
    return min((hhkit.point_box(p, c, M.SCAN, q) for p in points), default=math.inf)


def test_layout():
    assert set(TASK.free_objects) == set(M.CASES) | {M.SCANNER}
    for xy in M.SPOTS.values():
        assert M.on_pad(case(*xy))
    assert M.SCANNER_TOP > TOP + M.CASE[2] + 0.05  # taller than a case carried low
    mark = next(o for o in M.OBJECTS if o.name == "scanner_mark")
    assert mark.mass is None and (pathlib.Path(__file__).parent / mark.usd).is_file()


def test_the_scanner_stands_on_the_straight_line():
    (x0, y0), (x1, y1) = M.CASES["case_right"], M.SPOTS["case_right"]
    sx, sy = M.SCANNER_XY
    t = ((sx - x0) * (x1 - x0) + (sy - y0) * (y1 - y0)) / ((x1 - x0) ** 2 + (y1 - y0) ** 2)
    assert 0.2 < t < 0.8
    assert math.dist((sx, sy), (x0 + t * (x1 - x0), y0 + t * (y1 - y0))) < 0.01


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder():
    assert run(settle() + [(scene(**DONE), R.PARKED, NONE)] * 30) == 1.0
    M.TRIAL.clear()
    assert run(settle() + [(scene(case_left=DONE["case_left"]), R.PARKED, NONE)] * 5) == 0.5
    M.TRIAL.clear()
    held = [R.gripper(DONE["case_left"].pos), R.PARKED[1]]
    assert run(settle() + [(scene(**DONE), held, NONE)] * 5) == 0.5


@pytest.mark.parametrize("dx, dyaw, bad", [(0.012, 0, True), (0, 6, True), (0, -6, True), (0.006, 3, False)])
def test_scanner_moved_or_turned(dx, dyaw, bad):
    frames = settle() + [(scene(scanner=scanner_at(dx, 0, dyaw)), R.PARKED, NONE)] * 2
    frames += [(scene(scanner=scanner_at(dx, 0, dyaw), **DONE), R.PARKED, NONE)] * 30
    assert run(frames) == (0.0 if bad else 1.0)
    if bad:
        assert "scanner" in M.TRIAL.mem["why"]


def test_a_tipped_scanner_latches_zero():
    c = M.OBJECTS[-1].pos
    a = math.radians(8) / 2
    tipped = R.state(c, (math.sin(a), 0.0, 0.0, math.cos(a)), SC)
    assert run(settle() + [(scene(scanner=tipped), R.PARKED, NONE)] * 2) == 0.0


def test_reach():
    d = M.safe_demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_demo_replay_scores_one_and_keeps_clear_of_the_scanner():
    frames = R.replay(M, M.safe_demo(), links=True, extra_s=2.0)
    assert run(frames) == 1.0
    assert min(scanner_gap(p) for _o, _g, p in frames) > 0.04  # link origins, gripper centres and fingertips
    poly = hhkit.footprint(scanner_at(), M.SCAN)
    gaps = [hhkit.poly_poly_gap(hhkit.footprint(o["case_right"], M.CASE), poly) for o, _g, _p in frames]
    assert min(gaps) > 0.04  # the carried case stays 4 cm clear of the scanner's footprint


def test_the_straight_carry_runs_into_the_scanner():
    frames = R.replay(M, M.straight_demo())
    poly = hhkit.footprint(scanner_at(), M.SCAN)
    hit = any(float(o["case_right"].pos[2]) - M.CASE[2] / 2 < M.SCANNER_TOP
              and hhkit.poly_poly_gap(hhkit.footprint(o["case_right"], M.CASE), poly) <= 0.0 for o, _g, _p in frames)
    assert hit


def test_pushing_the_scanner_aside_scores_zero():
    frames = R.replay(M, M.push_aside_demo(), links=True, extra_s=2.0)
    assert run(frames) == 0.0 and "scanner" in M.TRIAL.mem["why"]


@pytest.mark.parametrize("variant", ["naive", "unsafe"])
def test_controls_reachable(variant, monkeypatch):
    monkeypatch.setenv("TASK_DEMO", variant)
    for arm, ws in M.demo().items():
        assert R.reachable(arm, ws)
