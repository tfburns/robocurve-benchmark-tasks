"""Offline checks of task.py's scorer (no simulator): python -m pytest -q test_task.py

The bat riding onto the sliding card cannot be replayed kinematically, so the scripted plans are checked in Isaac
Sim (see task.json "scores"); here the scorer is driven with hand-made object states."""

import math
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
NONE = np.zeros((0, 3))
LC = M.hhperson.local_centre(M.TUB)  # the simulator reports the tub's pos at its bounds' centre


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def tub(xy, rim, tilt_deg=0.0):
    q = (math.sin(math.radians(tilt_deg) / 2), 0.0, 0.0, math.cos(math.radians(tilt_deg) / 2))
    origin = np.array([xy[0], xy[1], M.TOP + rim + M.RIM_BELOW])
    return R.state(origin + R.hhkit.rot(q) @ LC, q, half=(M.TUB_X / 2, M.TUB_Y / 2, M.TUB_H / 2))


def bat(xy=None, z=None, vel=(0.0, 0.0, 0.0)):
    xy = M.BAT_XY if xy is None else np.asarray(xy, float)
    return R.state(np.array([xy[0], xy[1], M.TOP + M.BAT_R if z is None else z]),
                   half=(M.BAT_L / 2, M.BAT_R, M.BAT_R), vel=vel)


def card(xy):
    return R.state(np.array([xy[0], xy[1], M.TOP + M.CARD[2] / 2]), half=tuple(s / 2 for s in M.CARD))


def scene(tub_xy=None, rim=0.0, card_xy=None, on_card=False, bat_xy=None, tilt=0.0):
    st = R.initial_states(M)
    st["tub"] = tub(tub_xy if tub_xy is not None else M.TUB_START, rim, tilt)
    st["card"] = card(card_xy if card_xy is not None else M.CARD_START)
    st["bat"] = bat(bat_xy, z=(M.TOP + M.CARD[2] + M.BAT_R) if on_card else None)
    return st


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def sealed():
    return scene(tub_xy=M.BAT_XY, rim=M.CARD[2], card_xy=M.BAT_XY, on_card=True)


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))
    assert set(TASK.free_objects) == {"bat", "tub", "card"}


def test_sizes_are_full_scale():
    assert 0.11 <= M.BAT_L <= 0.13 and 0.015 <= M.BAT_KG <= 0.026  # a big brown bat
    assert M.TUB_X - 2 * M.WALL > M.BAT_L + 0.03 and M.TUB_Y - 2 * M.WALL > 2 * M.BAT_R + 0.03
    assert M.TUB_Y < 0.094  # the gripper opens 94 mm
    assert M.CARD[0] > M.TUB_X + 0.04 and M.CARD[1] > M.TUB_Y + 0.04
    assert M.GAP_MAX == pytest.approx(0.25 * 0.0254)


def test_start_scores_zero():
    assert step(R.initial_states(M)) == (False, 0.0)
    assert step(scene()) == (False, 0.0)


def test_rim_geometry():
    st = scene(tub_xy=M.BAT_XY, rim=0.004)
    assert np.allclose(M.rim_gaps(st), 0.004, atol=1e-6)
    assert M.bat_inside(st)
    st = scene(tub_xy=M.BAT_XY, rim=M.CARD[2], card_xy=M.BAT_XY, on_card=True)
    assert np.allclose(M.rim_gaps(st), 0.0, atol=1e-6) and M.card_cover(st) == 1.0


def test_sealed_scores_one():
    st = sealed()
    assert step(st) == (True, 1.0)


def test_tub_over_bat_without_card_is_quarter():
    assert step(scene(tub_xy=M.BAT_XY, rim=0.0)) == (False, 0.25)


def test_card_half_under_is_half():
    st = scene(tub_xy=M.BAT_XY, rim=0.003, card_xy=(M.BAT_XY[0], M.BAT_XY[1] + M.CARD[1] / 2 - 0.01), on_card=False)
    assert 0.5 <= M.card_cover(st) < 1.0
    assert step(st) == (False, 0.5)


def test_lifting_the_tub_off_the_bat_latches_zero():
    assert step(scene(tub_xy=M.BAT_XY, rim=0.0)) == (False, 0.25)
    assert step(scene(tub_xy=M.BAT_XY, rim=0.008)) == (False, 0.0)
    assert "lifted" in M.TRIAL.mem["why"]
    assert step(sealed()) == (False, 0.0)


def test_hovering_under_a_quarter_inch_is_allowed():
    assert step(scene(tub_xy=M.BAT_XY, rim=0.004)) == (False, 0.0)
    assert step(scene(tub_xy=M.BAT_XY, rim=0.006)) == (False, 0.0)
    assert not M.TRIAL.mem["violated"]
    assert step(sealed()) == (True, 1.0)


def test_tilted_tub_gap_counts_at_its_highest_corner():
    step(scene(tub_xy=M.BAT_XY, rim=0.0))
    step(scene(tub_xy=M.BAT_XY, rim=0.0, tilt=8.0))  # one long side up ~ 8.5 cm * sin 8 deg = 12 mm
    assert M.TRIAL.mem["violated"]


def test_touching_the_bat_latches_zero():
    st = scene()
    tcp = np.array([*M.BAT_XY, M.TOP + M.BAT_R + 0.013])
    assert step(st, grips=[R.PARKED[0], R.gripper(tcp)]) == (False, 0.0)
    assert "touched" in M.TRIAL.mem["why"]
    assert step(sealed(), grips=R.PARKED) == (False, 0.0)


def test_rim_on_the_bat_latches_zero():
    xy = (M.BAT_XY[0], M.BAT_XY[1] + M.TUB_Y / 2)  # the -y wall right over the bat's middle
    st = scene(tub_xy=xy, rim=2 * M.BAT_R)
    assert step(st) == (False, 0.0)
    assert "came down on the bat" in M.TRIAL.mem["why"]


def test_knocked_or_lost_bat_latches_zero():
    st = scene()
    st["bat"] = bat(vel=(0.8, 0.0, 0.0))
    assert step(st) == (False, 0.0)
    M.TRIAL.clear()
    st = scene(bat_xy=(M.BAT_XY[0] + 0.25, M.BAT_XY[1]))
    assert step(st) == (False, 0.0)
    assert "got away" in M.TRIAL.mem["why"]


def test_bat_out_from_under_after_cover_latches_zero():
    step(scene(tub_xy=M.BAT_XY, rim=0.0))
    assert step(scene(tub_xy=M.BAT_XY, rim=0.0, bat_xy=(M.BAT_XY[0], M.BAT_XY[1] + 0.06))) == (False, 0.0)
    assert "got out" in M.TRIAL.mem["why"]


def test_plans_cover_both_arms_and_reach():
    for name, fn in M.PLANS.items():
        plan = fn()
        assert plan and all(len(w) for w in plan.values()), name
        for side, wps in plan.items():
            for w in wps:
                assert w.pos[2] < 0.93 + M.TOP, (name, side, w.label)
    d = M.trap_demo()
    t_card_done = sum(w.duration for w in d["left"][:5])
    t_hold_end = sum(w.duration for w in d["right"][:9])
    assert t_hold_end > t_card_done  # the tub is held off the card until the card is in and the fingertips are off


def test_demo_hover_is_under_the_limit_with_margin():
    assert M.HOVER_RIM - M.SAG + 0.002 < M.GAP_MAX
    assert M.HOVER_RIM - M.SAG > M.CARD[2] + 0.002
