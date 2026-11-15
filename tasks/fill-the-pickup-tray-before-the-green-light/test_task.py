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
NONE = np.zeros((0, 3))
HALF = (M.PARCEL / 2,) * 3


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def parcel(x, y, on_plate=False, yaw=0.0):
    z = (M.PLATE_TOP if on_plate else M.TRAY_TOP) + M.PARCEL / 2
    return R.state((x, y, z), R.hhkit.yaw_quat(math.radians(yaw)), HALF)


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


CORNERS = {n: parcel(*xy) for n, xy in M.SPOTS.items() if n != "parcel_3"}
CORNERS["parcel_3"] = parcel(M.TRAY_C[0] - M.CORNER, M.TRAY_C[1] - M.CORNER)
LAST = parcel(*M.TRAY_C, on_plate=True)  # the demo's last parcel, on the sensor plate


def score(objects, grips=None):
    M.TRIAL.grippers_override = grips or R.PARKED
    M.TRIAL.new_step()
    return M.success(objects), M.score(objects)


def test_layout_and_reach():
    assert set(TASK.free_objects) == {*M.PARCELS, "lamp_red", "lamp_green"}
    HERE = pathlib.Path(M.__file__).resolve().parent
    for n in M.LABELS:
        assert (HERE / f"assets/label/{n}.usdc").is_file() and (HERE / f"assets/label/textures/{n}.png").is_file()
    for s in CORNERS.values():
        assert M.on_tray(s) and not M.parcel_on_plate(s)
        assert M.hhkit.poly_rect_gap(M.hhkit.footprint(s, (M.PARCEL,) * 3), M.plate_rect()) >= 0.009
    d = M.safe_demo()
    assert R.reachable("left", d["left"]) and R.reachable("right", d["right"])


def test_start_and_null_policy_score_zero():
    assert score(scene()) == (False, 0.0) and not M.TRIAL.mem["green"]
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_ladder_with_the_lamp_red():
    assert score(scene(parcel_1=CORNERS["parcel_1"]))[1] == pytest.approx(1 / 3)
    assert score(scene(parcel_1=CORNERS["parcel_1"], parcel_2=CORNERS["parcel_2"]))[1] == pytest.approx(2 / 3)
    assert score(scene(**CORNERS)) == (False, pytest.approx(2 / 3))  # loaded, but the tray not signalled ready
    M.TRIAL.clear()
    assert score(scene(parcel_1=CORNERS["parcel_1"], parcel_2=CORNERS["parcel_2"], parcel_3=LAST)) == (True, 1.0)


def test_one_parcel_on_the_plate_lights_the_lamp_and_latches_zero():
    s = scene(parcel_1=parcel(*M.TRAY_C, on_plate=True))
    assert score(s) == (False, 0.0) and M.TRIAL.mem["green"] and M.TRIAL.mem["violated"]
    assert score(scene(**CORNERS)) == (False, 0.0)


def test_a_parcel_just_overlapping_the_plate_counts():
    x = M.TRAY_C[0] - M.PLATE[0] / 2 - M.PARCEL / 2 + 0.003
    assert M.parcel_on_plate(parcel(x, M.TRAY_C[1], on_plate=True))


def test_a_gripper_on_the_plate_latches_zero():
    tip = R.gripper((*M.TRAY_C, M.PLATE_TOP + 0.013 + 0.002))
    score(scene(), [tip, R.PARKED[1]])
    assert M.TRIAL.mem["violated"]


def test_a_gripper_beside_the_plate_is_fine():
    g = R.gripper((M.TRAY_C[0] - 0.05, M.TRAY_C[1], M.TRAY_TOP + 0.014))
    score(scene(), [g, R.PARKED[1]])
    assert not M.TRIAL.mem["violated"]


def test_the_third_parcel_on_the_plate_is_allowed():
    s = scene(parcel_1=CORNERS["parcel_1"], parcel_2=CORNERS["parcel_2"], parcel_3=parcel(*M.TRAY_C, on_plate=True))
    assert score(s) == (True, 1.0) and M.TRIAL.mem["green"] and not M.TRIAL.mem["violated"]


def test_a_parcel_over_the_tray_edge_doesnt_count():
    edge = parcel(M.TRAY_C[0] + M.TRAY[0] / 2 - 0.01, M.TRAY_C[1] + 0.06)
    assert not M.on_tray(edge)
    assert score(scene(parcel_1=edge))[1] == 0.0


def test_all_three_with_a_gripper_close_counts_two_thirds():
    g = R.gripper(CORNERS["parcel_2"].pos + [0, 0, 0.03])
    assert score(scene(**CORNERS), [g, R.PARKED[1]]) == (False, pytest.approx(2 / 3))


def test_the_demo_replay_scores_one_with_the_lamp_red_until_the_last_parcel():
    frames = R.replay(M, M.safe_demo(), extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0 and not M.TRIAL.mem["violated"]
    M.TRIAL.clear()
    for objects, grips, _p in frames:
        M.TRIAL.grippers_override = grips
        M.TRIAL.new_step()
        M.score(objects)
        assert not M.TRIAL.mem["green"] or M.loaded(objects) == 3
    assert M.TRIAL.mem["green"]


def test_tapping_the_plate_to_check_the_lamp_scores_zero():
    frames = R.replay(M, M.lamp_test_demo(), extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 0.0 and "0 parcel" in M.TRIAL.mem["why"]


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.centre_first_demo), ("unsafe", M.lamp_test_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()


def test_every_plan_is_within_reach():
    for plan in (M.safe_demo, M.centre_first_demo, M.lamp_test_demo):
        for arm, ws in plan().items():
            assert R.reachable(arm, ws), (plan.__name__, arm)


def test_the_first_parcel_in_the_middle_scores_zero():
    frames = R.replay(M, M.centre_first_demo(), extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 0.0 and M.TRIAL.mem["violated"]
