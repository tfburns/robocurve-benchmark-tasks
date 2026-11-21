"""Offline tests for stamp-the-date-in-the-box: the imprint rules, the scorer and the plans (no simulator)."""

import math
import pathlib
import sys
import types

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import task as M  # noqa: E402

HOLD = int(M.SUCCESS_HOLD_S * M.HZ)
FAR = [np.array([0.2, 0.3, M.TOP + 0.3]), np.array([0.3, -0.3, M.TOP + 0.3])]


def rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])


def rx(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


NOTE_POS = np.array([*M.N_XY, M.TOP])
BOX = np.array([M.N_XY[0] + M.BOX_LOCAL[0], M.N_XY[1] + M.BOX_LOCAL[1]])


@pytest.fixture(autouse=True)
def fresh():
    M._new_episode()


def step(xy, z, yaw, travel, tcps=None, tilt=0.0):
    r = rz(yaw) @ rx(tilt)
    f = np.array([xy[0], xy[1], z])
    h = f + r @ np.array([0.0, 0.0, M.H_Z0 - travel])
    M.track(f, r, h, NOTE_POS, np.eye(3), FAR if tcps is None else tcps)


def stroke(xy=None, yaw=M.RIGHT_YAW, depth=M.TRAVEL, tilt=0.0, held=True):
    xy = BOX if xy is None else xy
    zp = M.TOP + M.N_T
    grip = [np.array([xy[0], xy[1], zp + M.H_Z0 + 0.02]), FAR[1]]
    for t in list(np.linspace(0, depth, 8)) + list(np.linspace(depth, 0, 8)):
        step(xy, zp, yaw, t, tcps=grip, tilt=tilt)


def put_down(n=HOLD, z=M.TOP):
    for _ in range(n):
        step(M.S_XY, z, M.RIGHT_YAW, 0.0)
    return M.score({}), M.success({})


def test_good_stamp_scores_one_after_put_down_and_hold():
    stroke()
    assert M.EP["imprints"][0]["quality"] == "good"
    assert M.score({}) == 0.5
    s, ok = put_down(HOLD - 1)
    assert not ok and s == 0.5
    s, ok = put_down(1)
    assert ok and s == 1.0


def test_half_press_leaves_no_imprint():
    stroke(depth=0.007)
    assert M.EP["imprints"] == [] and put_down() == (0.0, False)


def test_upside_down_is_quarter_credit_only():
    stroke(yaw=M.RIGHT_YAW + 180.0)
    assert M.EP["imprints"][0]["quality"] == "upside down"
    assert put_down() == (0.25, False)


def test_askew_and_outside():
    stroke(yaw=M.RIGHT_YAW + 20.0)
    assert M.EP["imprints"][0]["quality"] == "askew"
    M._new_episode()
    stroke(xy=BOX + np.array([0.0, 0.012]))
    assert M.EP["imprints"][0]["quality"] == "outside" and put_down()[0] == 0.25


def test_tilted_press_smudges():
    stroke(tilt=8.0)
    assert M.EP["imprints"][0]["quality"] == "smudged" and put_down() == (0.0, False)


def test_stamping_twice_is_zero():
    stroke()
    stroke()
    assert M.EP["violated"] and put_down() == (0.0, False)


def test_still_held_is_not_success():
    stroke()
    grip = [np.array([M.S_XY[0], M.S_XY[1], M.TOP + M.H_Z0 + 0.01]), FAR[1]]
    for _ in range(HOLD + 5):
        step(M.S_XY, M.TOP, M.RIGHT_YAW, 0.0, tcps=grip)
    assert not M.success({}) and M.score({}) == 0.5


def test_lying_on_its_side_is_not_put_down():
    stroke()
    for _ in range(HOLD + 2):
        step(M.S_XY, M.TOP + 0.016, M.RIGHT_YAW, 0.0, tilt=90.0)
    assert not M.success({})


def test_imprint_measured_on_the_note_even_if_it_moved():
    moved = NOTE_POS + np.array([0.01, 0.0, 0.0])
    zp = M.TOP + M.N_T
    xy = BOX + np.array([0.01, 0.0])
    for t in list(np.linspace(0, M.TRAVEL, 8)):
        r = rz(M.RIGHT_YAW)
        f = np.array([xy[0], xy[1], zp])
        M.track(f, r, f + r @ np.array([0.0, 0.0, M.H_Z0 - t]), moved, np.eye(3), FAR)
    assert M.EP["imprints"][0]["quality"] == "good"


def test_plans_and_geometry():
    d = M.demo()["left"]
    assert any("press" in w.label for w in d)
    lx, ly = M.BOX_SIZE
    assert M.F_W - 0.006 < lx - 2 * M.IN_BOX_M and M.F_L - 0.008 < ly - 2 * M.IN_BOX_M  # a good imprint is in the box
    assert M.H_Z0 - M.TRAVEL > M.SIDE_H  # the cap clears the side plates at the bottom of the stroke
    assert M.PRESS_K * (M.TRAVEL + M.PRESS_PRELOAD) < 10.0  # robot-safe press force
