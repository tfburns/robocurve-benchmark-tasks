"""On the station table, a two-station assembly cell. The left station's cell (green pad) holds a red part, the right
station's cell (green pad) a blue part. Between them runs a 12 cm grey transfer strip that both stations share: each
arm can reach its own cell and the strip, not the other station's cell, so parts change sides by being set down in
the strip. Only one station works in the strip at a time. The parts are loose; the pads and the strip are fixed.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Swap the red and blue parts between the two stations."
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import hhkit  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Swap the red and blue parts between the two stations."

PAD = (0.10, 0.10, 0.004)
PAD_TOP = TOP + PAD[2]
CELLS = {"left": (0.22, 0.20), "right": (0.22, -0.20)}
STRIP_W = 0.12  # the shared strip: |y| <= 6 cm, from the front of the bench to the back
STRIP = (0.22, 0.0, 0.24)  # x centre, y centre, length along x (marking only; the rule covers the full width)
STRIP_TOP = TOP + 0.002
PART = (0.05, 0.05, 0.04)
PART_KG = 0.15
PARTS = {"part_red": "left", "part_blue": "right"}
TARGET = {"part_red": "right", "part_blue": "left"}
COLORS = {"part_red": (0.80, 0.15, 0.12), "part_blue": (0.15, 0.30, 0.80)}

OBJECTS = [
    *[Box(f"cell_{n}", pos=(x, y, TOP + PAD[2] / 2), size=PAD, color=(0.30, 0.62, 0.35))
      for n, (x, y) in CELLS.items()],
    Box("transfer_strip", pos=(STRIP[0], STRIP[1], TOP + 0.001), size=(STRIP[2], STRIP_W, 0.002),
        color=(0.55, 0.56, 0.58)),
    *[Box(n, pos=(*CELLS[c], PAD_TOP + PART[2] / 2 + 0.0005), size=PART, mass=PART_KG, color=COLORS[n])
      for n, c in PARTS.items()],
]

TOUCH_M = 0.05
CONTACT_M = 0.006
RELEASE_M = 0.02


def _clear(mem):
    mem.update(violated=False, why="")


TRIAL = hhkit.Trial(on_clear=_clear)


def in_strip(p) -> bool:
    return abs(float(p[1]) - STRIP[1]) <= STRIP_W / 2


def _points(g):
    return (g["tcp"], *g["tips"])


def _track(objects) -> None:
    if not TRIAL.once() or TRIAL.mem["violated"]:
        return
    left, right = TRIAL.grippers()
    if any(in_strip(p) for p in _points(left)) and any(in_strip(p) for p in _points(right)):
        TRIAL.mem.update(violated=True, why="both arms were in the transfer strip at once")
    elif min(math.dist(a, b) for a in _points(left) for b in _points(right)) < TOUCH_M:
        TRIAL.mem.update(violated=True, why="the arms touched")


def on_cell(s, cell) -> bool:
    return (math.dist(s.pos[:2], CELLS[cell]) <= PAD[0] / 2
            and abs(float(s.pos[2]) - PART[2] / 2 - PAD_TOP) <= CONTACT_M)


def _released(s) -> bool:
    return all(hhkit.point_box(p, s.pos, PART, s.quat_xyzw) >= RELEASE_M
               for g in TRIAL.grippers() for p in _points(g))


def _ladder(objects) -> float:
    if TRIAL.mem["violated"]:
        return 0.0
    done = [n for n in PARTS if on_cell(objects[n], TARGET[n])]
    if len(done) == 2 and not all(_released(objects[n]) for n in PARTS):
        return 0.5
    return 0.5 * len(done)


def success(objects) -> bool:
    """Red on the right station's cell and blue on the left station's, both let go; never both arms in the strip at
    once and never touching. Held 1 s."""
    _track(objects)
    return bool(_ladder(objects) == 1.0 and all(objects[n].resting(0.02, 1.0) for n in PARTS))


def score(objects) -> float:
    """0 (latched) once a fingertip or gripper centre of each arm has been inside the transfer strip at the same
    moment, or the two grippers have come within 5 cm of each other. Otherwise 0.5 per part on its new cell; both
    count 1.0 once let go."""
    _track(objects)
    return _ladder(objects)


def distance(objects) -> float:
    _track(objects)
    return sum(math.dist(objects[n].pos[:2], CELLS[TARGET[n]]) for n in PARTS)


# -- scripted demo: the right arm puts blue in the strip; the left arm puts red in the strip and takes blue home;
# -- then the right arm takes red home. Never both in the strip.
TRAVEL_Z = TOP + 0.12
SPOTS = {"part_blue": (0.17, 0.0), "part_red": (0.27, 0.0)}  # set-down spots in the strip, 5 cm apart
HOME = {"left": (0.14, 0.22), "right": (0.14, -0.22)}


def _carry(src, dst, src_z, dst_z):
    (sx, sy), (dx, dy) = src, dst
    gz, pz = src_z + PART[2] / 2, dst_z + PART[2] / 2 + 0.004
    return [
        Waypoint(1.3, (sx, sy, TRAVEL_Z), FINGER_OPEN, "over the part"),
        Waypoint(1.0, (sx, sy, gz), FINGER_OPEN, "down"),
        Waypoint(0.6, (sx, sy, gz), FINGER_CLOSED, "grip"),
        Waypoint(0.9, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.3, (dx, dy, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.0, (dx, dy, pz), FINGER_CLOSED, "set down"),
        Waypoint(0.5, (dx, dy, pz), FINGER_OPEN, "let go"),
        Waypoint(0.8, (dx, dy, TRAVEL_Z), FINGER_OPEN, "up"),
    ]


def _home(side):
    return Waypoint(1.0, (*HOME[side], TRAVEL_Z), FINGER_OPEN, "home, out of the strip")


def _dur(ws):
    return sum(w.duration for w in ws)


def demo(early=0.0):
    r1 = [*_carry(CELLS["right"], SPOTS["part_blue"], PAD_TOP, STRIP_TOP), _home("right")]
    l1 = [*_carry(CELLS["left"], SPOTS["part_red"], PAD_TOP, STRIP_TOP),
          *_carry(SPOTS["part_blue"], CELLS["left"], STRIP_TOP, PAD_TOP), _home("left")]
    r2 = _carry(SPOTS["part_red"], CELLS["right"], STRIP_TOP, PAD_TOP) + [_home("right")]
    t_left = max(0.0, _dur(r1) - early)
    left = [Waypoint(t_left, (*HOME["left"], TRAVEL_Z), FINGER_OPEN, "wait for the right arm"), *l1] if t_left else l1
    wait_r = t_left + _dur(l1) - _dur(r1)
    right = [*r1, Waypoint(wait_r, (*HOME["right"], TRAVEL_Z), FINGER_OPEN, "wait for the left arm"), *r2]
    return {"left": left, "right": right}


def both_at_once_demo():
    """The negative control: the left arm sets off 7 s early and carries red into the strip while the right arm is
    still setting blue down there."""
    return demo(early=7.0)
