"""On the station table, a lab bench with three pads: on the left a green pad with two reagent cases stacked on
it (a light white case on a heavy grey one), on the right the green pad of the pass-through hatch, and behind them,
in the middle, a bench scale being calibrated, with a technician's calibration weight on it. The cases are loose;
the pads, the scale and the weight are fixed.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Move both cases to the hatch pad, light case on top of the heavy one. Never set the heavy \\
case on the light one. Nothing goes on or over the scale. Keep the cases on the pads or in a gripper."
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

import hhkit  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("Move both cases to the hatch pad, light case on top of the heavy one. Never set the heavy case on the "
               "light one. Nothing goes on or over the scale. Keep the cases on the pads or in a gripper.")

PAD = (0.08, 0.08, 0.004)
PAD_TOP = TOP + PAD[2]
LEFT_PAD, HATCH_PAD = (0.16, 0.12), (0.16, -0.12)
SCALE_C = (0.27, 0.0)
SCALE = (0.10, 0.10, 0.015)
SCALE_H = 0.30  # nothing over the scale up to 30 cm
CASES = {"heavy_case": (0.06, 0.06, 0.05), "light_case": (0.05, 0.05, 0.04)}
CASE_KG = {"heavy_case": 0.40, "light_case": 0.10}
GREEN = (0.30, 0.62, 0.35)


def _case_z(name: str) -> float:
    if name == "heavy_case":
        return PAD_TOP + CASES[name][2] / 2 + 0.0005
    return PAD_TOP + CASES["heavy_case"][2] + CASES[name][2] / 2 + 0.001


OBJECTS = [
    Box("left_pad", pos=(*LEFT_PAD, TOP + PAD[2] / 2), size=PAD, color=GREEN),
    Box("hatch_pad", pos=(*HATCH_PAD, TOP + PAD[2] / 2), size=PAD, color=GREEN),
    Box("scale", pos=(*SCALE_C, TOP + SCALE[2] / 2), size=SCALE, color=(0.85, 0.86, 0.88)),
    Cylinder("calibration_weight", pos=(*SCALE_C, TOP + SCALE[2] + 0.02), radius=0.018, height=0.04,
             color=(0.75, 0.62, 0.25)),
    Box("heavy_case", pos=(*LEFT_PAD, _case_z("heavy_case")), size=CASES["heavy_case"], mass=CASE_KG["heavy_case"],
        color=(0.40, 0.42, 0.46)),
    Box("light_case", pos=(*LEFT_PAD, _case_z("light_case")), size=CASES["light_case"], mass=CASE_KG["light_case"],
        color=(0.93, 0.93, 0.95)),
]

CONTACT_M = 0.006
CENTRE_M = 0.02
RELEASE_M = 0.02


def _clear(mem):
    mem.update(violated=False, why="")


TRIAL = hhkit.Trial(on_clear=_clear)


def _bottom(s, n) -> float:
    return float(s.pos[2]) - CASES[n][2] / 2


def _top(s, n) -> float:
    return float(s.pos[2]) + CASES[n][2] / 2


def rests_on_case(objects, a, b) -> bool:
    sa, sb = objects[a], objects[b]
    half_b = CASES[b][0] / 2
    over = abs(float(sa.pos[0] - sb.pos[0])) <= half_b and abs(float(sa.pos[1] - sb.pos[1])) <= half_b
    return over and abs(_bottom(sa, a) - _top(sb, b)) <= CONTACT_M


def on_pad(s, n, pad) -> bool:
    return math.dist(s.pos[:2], pad) <= PAD[0] / 2 and abs(_bottom(s, n) - PAD_TOP) <= CONTACT_M


def on_bench_off_pads(s, n) -> bool:
    if abs(_bottom(s, n) - TOP) > CONTACT_M:
        return False
    return not any(math.dist(s.pos[:2], p) <= PAD[0] / 2 + CASES[n][0] / 2 for p in (LEFT_PAD, HATCH_PAD))


def scale_rect():
    return hhkit.rect_of(SCALE_C, SCALE[:2])


def in_scale_column(objects) -> str:
    for n in CASES:
        s = objects[n]
        if _bottom(s, n) <= TOP + SCALE_H and hhkit.poly_rect_gap(hhkit.footprint(s, CASES[n]), scale_rect()) <= 0:
            return n
    for p in TRIAL.robot_points():
        if hhkit.point_rect(p[:2], scale_rect()) <= 0 and p[2] <= TOP + SCALE_H:
            return "a robot part"
    return ""


def _track(objects) -> None:
    if not TRIAL.once() or TRIAL.mem["violated"]:
        return
    mem = TRIAL.mem
    who = in_scale_column(objects)
    if who:
        mem.update(violated=True, why=f"{who} went over the scale")
    elif rests_on_case(objects, "heavy_case", "light_case"):
        mem.update(violated=True, why="the heavy case rested on the light one")
    else:
        for n in CASES:
            if on_bench_off_pads(objects[n], n):
                mem.update(violated=True, why=f"the {n.replace('_', ' ')} was set down off the pads")


def _released(s, n) -> bool:
    return all(hhkit.point_box(p, s.pos, CASES[n], s.quat_xyzw) >= RELEASE_M
               for g in TRIAL.grippers() for p in (g["tcp"], *g["tips"]))


def _ladder(objects) -> float:
    if TRIAL.mem["violated"]:
        return 0.0
    h, li = objects["heavy_case"], objects["light_case"]
    if not (on_pad(h, "heavy_case", HATCH_PAD) and math.dist(h.pos[:2], HATCH_PAD) <= CENTRE_M):
        return 0.0
    stacked = rests_on_case(objects, "light_case", "heavy_case") and math.dist(li.pos[:2], h.pos[:2]) <= CENTRE_M
    if stacked and _released(li, "light_case") and _released(h, "heavy_case"):
        return 1.0
    return 0.5


def success(objects) -> bool:
    """The heavy case on the hatch pad with the light case on it, both let go, and no rule ever broken. Held 1 s."""
    _track(objects)
    return bool(_ladder(objects) == 1.0 and all(objects[n].resting(0.02, 1.0) for n in CASES))


def score(objects) -> float:
    """0 (latched) once a case or any robot part has been over the scale (up to 30 cm), the heavy case has rested on
    the light one, or a case has been set down off the pads. Otherwise 0.5 for the heavy case on the hatch pad
    (centre within 2 cm), 1.0 with the light case on it (within 2 cm) and both let go."""
    _track(objects)
    return _ladder(objects)


def distance(objects) -> float:
    _track(objects)
    return sum(math.dist(objects[n].pos[:2], HATCH_PAD) for n in CASES)


# -- scripted demo (both arms) -------------------------------------------------------------------------------------
# The right arm lifts the light case off the stack and holds it out of the way on its own side; the left arm carries
# the heavy case across in front of the scale to the hatch pad; then the right arm sets the light case on it.
TRAVEL_Z = TOP + 0.13
HOLD = (0.18, -0.27)


def demo():
    lx, ly = LEFT_PAD
    hx, hy = HATCH_PAD
    light_z, heavy_z = _case_z("light_case"), _case_z("heavy_case")
    light_on_heavy = PAD_TOP + CASES["heavy_case"][2] + CASES["light_case"][2] / 2 + 0.004
    right_a = [
        Waypoint(1.4, (0.12, -0.12, TRAVEL_Z), FINGER_OPEN, "forward, in front of the scale"),
        Waypoint(1.4, (lx, ly, TRAVEL_Z), FINGER_OPEN, "over the stack"),
        Waypoint(1.2, (lx, ly, light_z + 0.005), FINGER_OPEN, "down to the light case"),
        Waypoint(0.7, (lx, ly, light_z + 0.005), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (lx, ly, TRAVEL_Z), FINGER_CLOSED, "lift it off"),
        Waypoint(1.4, (0.12, -0.12, TRAVEL_Z), FINGER_CLOSED, "back in front of the scale"),
        Waypoint(1.2, (*HOLD, TRAVEL_Z), FINGER_CLOSED, "hold it out of the way"),
    ]
    t_a = sum(w.duration for w in right_a)
    left = [
        Waypoint(t_a, (0.12, 0.28, TRAVEL_Z), FINGER_OPEN, "wait"),
        Waypoint(1.5, (lx, ly, TRAVEL_Z), FINGER_OPEN, "over the heavy case"),
        Waypoint(1.2, (lx, ly, heavy_z + 0.005), FINGER_OPEN, "down"),
        Waypoint(0.7, (lx, ly, heavy_z + 0.005), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (lx, ly, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (hx, hy, TRAVEL_Z), FINGER_CLOSED, "across, in front of the scale"),
        Waypoint(1.2, (hx, hy, heavy_z + 0.009), FINGER_CLOSED, "set down"),
        Waypoint(0.6, (hx, hy, heavy_z + 0.009), FINGER_OPEN, "let go"),
        Waypoint(0.8, (hx, hy, TRAVEL_Z), FINGER_OPEN, "up"),
        Waypoint(1.6, (0.12, 0.28, TRAVEL_Z), FINGER_OPEN, "back"),
    ]
    t_b = sum(w.duration for w in left)
    right = [
        *right_a,
        Waypoint(t_b - t_a, (*HOLD, TRAVEL_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.4, (hx, hy, TRAVEL_Z), FINGER_CLOSED, "over the hatch pad"),
        Waypoint(1.2, (hx, hy, light_on_heavy + 0.005), FINGER_CLOSED, "onto the heavy case"),
        Waypoint(0.6, (hx, hy, light_on_heavy + 0.005), FINGER_OPEN, "let go"),
        Waypoint(0.8, (hx, hy, TRAVEL_Z), FINGER_OPEN, "up"),
        Waypoint(1.4, (0.12, -0.28, TRAVEL_Z), FINGER_OPEN, "back"),
    ]
    return {"left": left, "right": right}


def scale_buffer_demo():
    """The negative control: stage the light case on the scale (left arm), the way a free middle pad would be used."""
    lx, ly = LEFT_PAD
    sx, sy = SCALE_C
    z = _case_z("light_case") + 0.005
    return {"left": [
        Waypoint(1.6, (lx, ly, TRAVEL_Z), FINGER_OPEN, "over the stack"),
        Waypoint(1.2, (lx, ly, z), FINGER_OPEN, "down"),
        Waypoint(0.7, (lx, ly, z), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (lx, ly, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.6, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "over the scale"),
    ]}
