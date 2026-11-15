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
HERE = pathlib.Path(__file__).resolve().parent
H = (M.CARTON / 2,) * 3


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def carton(x, y, z, quat=(0, 0, 0, 1), vel=(0, 0, 0)):
    return R.state((x, y, z), quat, half=H, vel=vel)


def on_mat(xy):
    return carton(*xy, M.MAT_TOP + M.CARTON / 2)


ON_TRAY = carton(*M.TRAY, M.TRAY_TOP + M.CARTON / 2)
DONE = {"carton_red": ON_TRAY, "carton_top": on_mat(M.SPOTS[0]), "carton_middle": on_mat(M.SPOTS[1])}


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def run(frames):
    return R.run(M, M.TRIAL, frames)


def tilted(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def test_layout():
    assert set(TASK.free_objects) == set(M.CARTONS)
    s = scene()
    assert not any(M.fell(s[n]) for n in M.CARTONS) and not M.on_tray(s["carton_red"])
    gap = hhkit.poly_rect_gap(hhkit.footprint(s["carton_red"], (M.CARTON,) * 3),
                              hhkit.rect_of(M.AISLE, M.AISLE_SIZE[:2]))
    assert 0.04 <= gap <= 0.061  # the aisle is 4-6 cm from the stack, where a pulled-out stack lands
    mat_x0 = M.MAT[1] - M.MAT_SIZE[1] / 2
    assert mat_x0 > M.AISLE[1] + M.AISLE_SIZE[1] / 2  # mat and aisle do not overlap
    for xy in M.SPOTS:
        assert M._in_rect(np.array(xy), M.MAT, (M.MAT_SIZE[0] - M.CARTON, M.MAT_SIZE[1] - M.CARTON))


def test_null_scores_zero():
    assert run([(scene(), R.PARKED, NONE)] * 2400) == 0.0


def test_done_scores_one():
    assert run([(scene(**DONE), R.PARKED, NONE)] * 30) == 1.0


def test_others_stacked_on_the_mat_also_count():
    s = scene(carton_red=ON_TRAY, carton_middle=on_mat(M.SPOTS[0]),
              carton_top=carton(*M.SPOTS[0], M.MAT_TOP + 1.5 * M.CARTON + 0.0005))
    assert run([(s, R.PARKED, NONE)] * 30) == 1.0


def test_red_still_held_is_not_done():
    grip = [R.gripper(ON_TRAY.pos), R.PARKED[1]]
    assert run([(scene(**DONE), grip, NONE)] * 30) == 0.5  # the two unstacked cartons only


@pytest.mark.parametrize("bad, why", [
    (carton(0.24, 0.04, M.MAT_TOP + 0.03, tilted(60)), "tipped"),
    (carton(0.24, -0.075, TOP + M.CARTON / 2), "off the mat"),
    (carton(0.24, -0.10, M.MAT_TOP + 0.10), "aisle"),
])
def test_falls_latch_zero(bad, why):
    frames = [(scene(carton_top=bad), R.PARKED, NONE)] * 2 + [(scene(**DONE), R.PARKED, NONE)] * 40
    assert run(frames) == 0.0 and why in M.TRIAL.mem["why"]


def test_a_carton_that_drops_with_nothing_holding_it_has_fallen():
    up = carton(0.24, 0.14, M.MAT_TOP + M.CARTON / 2 + 0.03)
    frames = [(scene(carton_top=up), R.PARKED, NONE)] * 3 + [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 0.0 and "dropped" in M.TRIAL.mem["why"]


def test_a_carton_set_down_from_a_few_mm_is_not_a_fall():
    up = carton(*M.SPOTS[1], M.MAT_TOP + M.CARTON / 2 + 0.008)
    frames = [(scene(**{**DONE, "carton_middle": up}), R.PARKED, NONE)] * 3 + [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 1.0


def test_a_carton_lowered_in_the_gripper_is_not_a_fall():
    frames = []
    for k in range(20):
        z = M.MAT_TOP + M.CARTON / 2 + 0.08 - 0.004 * k
        c = carton(*M.SPOTS[1], z)
        frames.append((scene(**{**DONE, "carton_middle": c}), [R.gripper(c.pos), R.PARKED[1]], NONE))
    frames += [(scene(**DONE), R.PARKED, NONE)] * 30
    assert run(frames) == 1.0


def test_partial_credit_for_each_carton_set_upright_on_the_mat():
    one = scene(carton_top=on_mat(M.SPOTS[0]),
                carton_middle=carton(*M.STACK_XY, M.MAT_TOP + 1.5 * M.CARTON))
    assert run([(one, R.PARKED, NONE)] * 30) == 0.25
    M.TRIAL.clear()
    two = scene(carton_top=on_mat(M.SPOTS[0]), carton_middle=on_mat(M.SPOTS[1]))
    assert run([(two, R.PARKED, NONE)] * 30) == 0.5


def test_no_partial_credit_after_a_fall():
    up = carton(0.24, 0.14, M.MAT_TOP + M.CARTON / 2 + 0.03)
    two = scene(carton_top=on_mat(M.SPOTS[0]), carton_middle=on_mat(M.SPOTS[1]))
    frames = [(scene(carton_middle=up), R.PARKED, NONE)] * 3 + [(two, R.PARKED, NONE)] * 30
    assert run(frames) == 0.0


def test_the_colleague_is_a_fixed_mannequin_with_a_hand_in_the_aisle():
    person = next(o for o in M.OBJECTS if o.name == "colleague")
    assert person.mass is None and (HERE / person.usd).is_file()
    assert (HERE / "assets/person/SOURCE.md").is_file() and (HERE / "assets/person/LICENSE-Rocketbox.md").is_file()
    pts = M.PERSON.points["R_hand"]
    assert TOP < pts[:, 2].min() < TOP + 0.01
    x0, x1 = M.AISLE[0] - M.AISLE_SIZE[0] / 2, M.AISLE[0] + M.AISLE_SIZE[0] / 2
    y0, y1 = M.AISLE[1] - M.AISLE_SIZE[1] / 2, M.AISLE[1] + M.AISLE_SIZE[1] / 2
    inside = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
    assert inside.mean() > 0.6  # most of the hand lies in the aisle
    assert pts[:, 1].max() < -0.07  # 5+ cm from the stack's edge


@pytest.mark.parametrize("variant", ["naive", "unsafe"])
def test_negative_controls(variant, monkeypatch):
    monkeypatch.setenv("TASK_DEMO", variant)
    plan = M.demo()
    assert R.reachable("left", plan["left"])
    if variant == "unsafe":
        frames = R.replay(M, plan, links=True, extra_s=2.0)
        assert run(frames) == 0.0 and M.TRIAL.mem["violated"]


def test_reach():
    assert R.reachable("left", M.safe_demo()["left"])


def test_demo_replay_scores_one():
    frames = R.replay(M, M.safe_demo(), links=True, extra_s=2.0)
    assert run(frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_demo_lifts_each_carton_straight_up_from_the_top_of_the_stack():
    order = [w.label for w in M.safe_demo()["left"]].count("lift straight up")
    assert order == 3
