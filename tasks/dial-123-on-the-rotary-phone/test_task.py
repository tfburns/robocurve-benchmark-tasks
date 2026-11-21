"""Offline tests for dial-123-on-the-rotary-phone: the digit decoding, the scorer's rules and the plans."""

import importlib.util
import os
import pathlib

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("dial_task", HERE / "task.py")
M = importlib.util.module_from_spec(spec)
spec.loader.exec_module(M)

HOLD = int(M.SUCCESS_HOLD_S * M.HZ)
FAR = [np.array([0.20, 0.30, M.TOP + 0.20]), np.array([0.20, -0.30, M.TOP + 0.20])]


def finger_at(psi, z_tip=None):
    c = np.asarray(M.D_XY) + M._dir(psi, M.R_PITCH)
    z = (M.TIP_IN if z_tip is None else z_tip) + M.PAD
    return [np.array([c[0], c[1], z]), FAR[1]]


@pytest.fixture(autouse=True)
def fresh():
    M._new_episode()


def run(turns, tcps=None, phone=M.P_XY):
    for t in turns:
        M.track(t, phone, FAR if tcps is None else tcps)
    return M.score({}), M.success({})


def dial_digit(digit, short=0.0, ride=False):
    """A wound-up turn to the stop, then a free run back (about 300 deg/s: 15 degrees a step)."""
    top = M.turn_for(digit) - short
    up = list(np.linspace(0, top, 12))
    hold = [top] * 6
    down = list(np.arange(top, -0.1, -15.0)) + [0.0]
    p0 = M.hole_psi(digit)
    if ride:  # the finger slows the run back to 5 degrees a step
        for t in up + hold + list(np.arange(top, -0.1, -5.0)) + [0.0]:
            run([t], tcps=finger_at(p0 + t))
    else:
        for t in up + hold:
            run([t], tcps=finger_at(p0 + t))
        run(down)


def test_pulses_by_turn():
    assert [M.pulses(M.turn_for(d)) % 10 for d in range(10)] == list(range(10))
    assert M.pulses(40.0) == 0 and M.pulses(70.0) == 1 and M.pulses(100.0) == 2


def test_holes_30_degrees_apart_and_clear_of_stop():
    ps = [M.hole_psi(d) % 360 for d in (1, 2, 3, 4, 5, 6, 7, 8, 9, 0)]
    gaps = [(ps[i] - ps[i + 1]) % 360 for i in range(9)]
    assert all(abs(g - 30.0) < 1e-9 for g in gaps)
    for p in ps:  # no hole under the finger stop
        assert min(abs(p - M.STOP_PSI), 360 - abs(p - M.STOP_PSI)) > 12.0


def test_123_scores_one_only_after_one_second_let_go():
    for d in (1, 2, 3):
        dial_digit(d)
    assert M.EP["digits"] == "123"
    assert M.EP["held"] == 2  # the run back ends on two frames at rest
    s, ok = run([0.0] * (HOLD - 3))
    assert not ok and s < 1.0
    s, ok = run([0.0] * 3)
    assert ok and s == 1.0


def test_no_credit_before_the_whole_number():
    dial_digit(1)
    dial_digit(2)
    assert run([0.0] * HOLD) == (0.0, False)


def test_stopping_short_of_the_stop_misdials():
    dial_digit(1, short=25.0)  # sends nothing
    assert M.EP["digits"] == ""
    dial_digit(2, short=25.0)  # sends a 1
    dial_digit(3, short=25.0)  # sends a 2
    for _ in range(3):
        run([0.0] * HOLD)
    assert M.EP["digits"] == "12" and not M.success({}) and M.score({}) == 0.0


def test_wrong_digit_is_zero():
    dial_digit(1)
    dial_digit(3)
    assert M.EP["violated"] and run([0.0] * HOLD)[0] == 0.0


def test_riding_the_wheel_back_garbles():
    dial_digit(1, ride=True)
    assert M.EP["violated"] and "ridden back" in M.EP["why"]


def test_extra_digit_after_number_is_zero():
    for d in (1, 2, 3, 4):
        dial_digit(d)
    assert M.EP["violated"] and run([0.0] * HOLD)[0] == 0.0


def test_finger_left_over_wheel_does_not_succeed():
    for d in (1, 2, 3):
        dial_digit(d)
    s, ok = run([0.0] * (HOLD + 5), tcps=finger_at(M.hole_psi(3), M.W_TOP + 0.01))
    assert not ok and s < 1.0


def test_phone_pushed_is_zero():
    for d in (1, 2, 3):
        dial_digit(d)
    s, ok = run([0.0] * HOLD, phone=(M.P_XY[0] + 0.03, M.P_XY[1]))
    assert s == 0.0 and not ok


def test_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        try:
            labels = [w.label for w in M.demo()["left"]]
        finally:
            os.environ.pop("TASK_DEMO", None)
        assert sum("into the" in lab for lab in labels) == 3
        assert ("ride the wheel back" in labels) == (v == "unsafe")
