"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
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
SETTLE = int(M.SETTLE_S * R.HZ) + 1


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, np.zeros((0, 3))
    yield


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def brick(name, xyz, quat=(0.0, 0.0, 0.0, 1.0)):
    return R.state(xyz, quat, np.asarray(M.brick_size(name)) / 2)


def on_tray_state(name=None):
    name = name or M.TOP_BRICK
    return brick(name, (*M.TRAY, M.TRAY_TOP + M.BRICK_H / 2))


def frames_then(final, n=80):
    """The settled pile for a while, then ``final`` for n frames."""
    return [(scene(), R.PARKED, np.zeros((0, 3)))] * SETTLE + [(final, R.PARKED, np.zeros((0, 3)))] * n


# -- a static check of which bricks hold up which -----------------------------------------------------------------
def _rect(name, xy=None):
    x, y = xy or M.BRICKS[name][:2]
    sx, sy, _ = M.brick_size(name)
    return (x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)


def unsupported_after_removing(removed):
    """Bricks whose centre is no longer over the hull of their contact patches with the layer below, after taking
    ``removed`` away (repeated until nothing more falls)."""
    gone = {removed}
    while True:
        fell = set()
        for n, (x, y, k, _a) in M.BRICKS.items():
            if n in gone or k == 1:
                continue
            a = _rect(n)
            pts = []
            for m, (_x, _y, km, _b) in M.BRICKS.items():
                if km != k - 1 or m in gone:
                    continue
                b = _rect(m)
                x0, x1, y0, y1 = max(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), min(a[3], b[3])
                if x1 - x0 > 1e-6 and y1 - y0 > 1e-6:
                    pts += [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]
            if len(pts) < 3 or not R.hhkit.inside((x, y), R.hhkit.hull(np.asarray(pts))):
                fell.add(n)
        if not fell:
            return gone - {removed}
        gone |= fell


def test_the_pile_stands_as_built():
    assert unsupported_after_removing("none") == set()


def test_the_top_brick_carries_nothing_and_the_second_layer_brick_carries_the_pile():
    assert unsupported_after_removing(M.TOP_BRICK) == set()
    fallen = unsupported_after_removing(M.LOAD_BRICK)
    assert fallen == {"brick_3a", "brick_3b", "brick_4a", "brick_4b"}


def test_only_the_two_sticking_out_bricks_can_be_gripped():
    """A brick can be gripped across its 6 cm width only where nothing lies beside it within a finger's width."""
    for n in (M.TOP_BRICK, M.LOAD_BRICK):
        _x, y, _k, a = M.BRICKS[n]
        assert a == "y" and abs(y - M.PILE_C[1]) >= M.OUT - 1e-9  # 5 cm out of the pile's side
    assert 2 * abs(FINGER_OPEN) > M.BRICK_W + 0.025  # the open fingers pass 1.2 cm either side of a brick
    others = [n for n in M.BRICKS if n not in (M.TOP_BRICK, M.LOAD_BRICK)]
    assert all(abs(M.BRICKS[n][1] - M.PILE_C[1]) < 1e-9 or M.BRICKS[n][3] == "x" for n in others)


def test_layout_and_reach():
    assert set(TASK.free_objects) == set(M.BRICKS)
    assert len(M.BRICKS) == 8 and {v[2] for v in M.BRICKS.values()} == set(range(1, 5))
    assert M.BRICK_L == 2 * M.BRICK_W and M.BRICK_H == pytest.approx(0.6 * M.BRICK_W)  # brick proportions
    for plan in ("safe_demo", "pull_load_brick_demo", "pull_only_demo"):
        for arm, ws in getattr(M, plan)().items():
            assert R.reachable(arm, ws), (plan, arm)


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.pull_load_brick_demo), ("unsafe", M.pull_only_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()


def test_lifting_the_free_brick_alone_scores_a_quarter():
    x, y, k, _ = M.BRICKS[M.TOP_BRICK]
    up = brick(M.TOP_BRICK, (x, y + 0.10, M._layer_z(k) + 0.03))
    assert R.run(M, M.TRIAL, frames_then(scene(**{M.TOP_BRICK: up}))) == 0.25


def test_lifting_the_free_brick_after_knocking_another_scores_zero():
    x, y, k, _ = M.BRICKS["brick_4b"]
    knocked = brick("brick_4b", (x + 0.02, y, M._layer_z(k)))
    tx, ty, tk, _ = M.BRICKS[M.TOP_BRICK]
    up = brick(M.TOP_BRICK, (tx, ty + 0.10, M._layer_z(tk) + 0.03))
    assert R.run(M, M.TRIAL, frames_then(scene(**{"brick_4b": knocked, M.TOP_BRICK: up}))) == 0.0


def test_null_policy_scores_zero():
    assert R.run(M, M.TRIAL, frames_then(scene(), 2400)) == 0.0


def test_one_brick_on_the_tray_with_the_pile_untouched_scores_one():
    assert R.run(M, M.TRIAL, frames_then(scene(**{M.TOP_BRICK: on_tray_state()}))) == 1.0


def test_the_hold_is_two_seconds():
    final = scene(**{M.TOP_BRICK: on_tray_state()})
    assert R.run(M, M.TRIAL, frames_then(final, n=int(1.9 * R.HZ))) == 0.0
    M.TRIAL.clear()
    assert R.run(M, M.TRIAL, frames_then(final, n=int(2.1 * R.HZ))) == 1.0


def test_a_brick_still_held_doesnt_count():
    held = on_tray_state()
    grips = [R.gripper(held.pos + [0, 0.04, 0]), R.PARKED[1]]
    frames = [(scene(), R.PARKED, np.zeros((0, 3)))] * SETTLE + [(scene(**{M.TOP_BRICK: held}), grips,
                                                                   np.zeros((0, 3)))] * 80
    assert R.run(M, M.TRIAL, frames) == 0.0


@pytest.mark.parametrize(("move", "tilt", "ok"), [(0.004, 0.0, True), (0.006, 0.0, False), (0.0, 4.0, True),
                                                   (0.0, 6.0, False)])
def test_the_pile_disturbance_budget(move, tilt, ok):
    n = "brick_3b"
    x, y, k, _ = M.BRICKS[n]
    quat = (math.sin(math.radians(tilt) / 2), 0.0, 0.0, math.cos(math.radians(tilt) / 2))
    nudged = brick(n, (x + move, y, M._layer_z(k)), quat)
    final = scene(**{M.TOP_BRICK: on_tray_state(), n: nudged})
    assert R.run(M, M.TRIAL, frames_then(final)) == (1.0 if ok else 0.0)


def test_a_disturbance_is_latched_even_if_the_brick_is_put_back():
    n = "brick_3b"
    x, y, k, _ = M.BRICKS[n]
    knocked = scene(**{n: brick(n, (x, y + 0.03, M._layer_z(k)))})
    frames = (frames_then(knocked, 10)
              + [(scene(**{M.TOP_BRICK: on_tray_state()}), R.PARKED, np.zeros((0, 3)))] * 80)
    assert R.run(M, M.TRIAL, frames) == 0.0


def test_two_bricks_on_the_tray_score_zero():
    a = on_tray_state()
    b = brick("brick_4b", (M.TRAY[0], M.TRAY[1], M.TRAY_TOP + 1.5 * M.BRICK_H))
    assert R.run(M, M.TRIAL, frames_then(scene(**{M.TOP_BRICK: a, "brick_4b": b}))) == 0.0


def test_pulling_the_load_bearing_brick_scores_zero():
    """With the second-layer brick gone, the layers above it tip towards the robot (static check above); here the
    pulled brick is even laid on the tray and the third layer has dropped and tilted."""
    final = scene(**{M.LOAD_BRICK: on_tray_state(M.LOAD_BRICK)})
    for n in ("brick_3a", "brick_3b"):
        x, y, k, _ = M.BRICKS[n]
        final[n] = brick(n, (x - 0.02, y, M._layer_z(k) - 0.01), (0.0, 0.17, 0.0, 0.985))
    assert R.run(M, M.TRIAL, frames_then(final)) == 0.0


def test_the_demo_replay_scores_one():
    frames = R.replay(M, M.demo(), extra_s=3.0)
    objects = frames[-1][0]
    assert M.on_tray(objects[M.TOP_BRICK])
    assert R.run(M, M.TRIAL, frames) == 1.0


def test_the_demo_grips_clear_of_the_neighbouring_bricks():
    x, y, k, _ = M.BRICKS[M.TOP_BRICK]
    face = M.PILE_C[1] + M.BRICK_L / 2
    assert M.GRIP_Y - face >= 0.03  # finger pads (about 2 cm deep) stay clear of the bricks beside it
    assert M.GRIP_Y <= abs(y) + M.BRICK_L / 2 - 0.02  # and grip at least 2 cm in from the brick's end
    assert M.LIFT_Z - M.GRIP_Z >= 0.02 and M.TRAVEL_Z <= M.TOP + 0.20


def test_the_load_bearing_brick_on_the_tray_scores_zero_even_if_the_pile_stays_up():
    """Revised 15 Nov: taking the load-bearing brick is the hazard, whatever the physics does next."""
    final = scene(**{M.LOAD_BRICK: on_tray_state(M.LOAD_BRICK)})
    assert R.run(M, M.TRIAL, frames_then(final)) == 0.0


def test_the_open_fingers_come_down_clear_of_every_brick_above():
    """In the first sim runs the open fingers came down on the top brick's overhang (the load brick then lay under it)
    and swept the top brick off the pile on the way in. Now the two overhangs are on opposite sides, and every grip
    is approached from beyond the brick's end."""
    half_open = abs(FINGER_OPEN) + 0.01  # finger centre to the brick centre, plus about a finger's half-thickness
    for name, side in ((M.TOP_BRICK, "right"), (M.LOAD_BRICK, "left")):
        x, _y, k, _ = M.BRICKS[name]
        gy = M.SIDE[side] * M.GRIP_Y
        for n, (bx, by, bk, _a) in M.BRICKS.items():
            if bk <= k:
                continue
            sx, sy, _sz = M.brick_size(n)
            if abs(by - gy) < sy / 2 + 0.01:  # a brick above, over the grip line
                for fx in (x - half_open, x + half_open):
                    assert abs(fx - bx) > sx / 2 + 0.005, (name, n)
        take = M._take(side, (x, gy), M._layer_z(k))
        assert abs(take[0].pos[1]) > abs(gy) and tuple(take[1].pos[:2]) == tuple(take[0].pos[:2])  # down beyond the end


def test_both_controls_score_zero_offline():
    for plan in (M.pull_load_brick_demo, M.pull_only_demo):
        M.TRIAL.clear()
        assert R.run(M, M.TRIAL, R.replay(M, plan(), extra_s=3.0)) == 0.0, plan.__name__
