"""On the station table, between the arms, three square pallet spots are marked on the bench in a row across the
robot's view: left, middle and right. Three paint tins (closed, solid: 6, 5 and 4 cm across, 3 cm tall) stand
stacked on the left spot, largest at the bottom. The tins are loose; the spots are fixed.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Move the three tins from the left spot to the right spot, one tin at a time, never putting \\
a bigger tin on a smaller one. Use only the three marked spots."
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
INSTRUCTION = ("Move the three tins from the left spot to the right spot, one tin at a time, never putting a bigger "
               "tin on a smaller one. Use only the three marked spots.")

SPOT_X = 0.24
SPOTS = {"left": (SPOT_X, 0.10), "middle": (SPOT_X, 0.0), "right": (SPOT_X, -0.10)}
SPOT_SIZE = (0.085, 0.085, 0.004)
SPOT_TOP = TOP + SPOT_SIZE[2]
TIN_H = 0.03
TINS = {"tin_large": 0.030, "tin_medium": 0.025, "tin_small": 0.020}  # radius, m
TIN_KG = {"tin_large": 0.16, "tin_medium": 0.12, "tin_small": 0.08}
ORDER = ("tin_large", "tin_medium", "tin_small")  # bottom to top
TIN_COLOR = {"tin_large": (0.20, 0.38, 0.70), "tin_medium": (0.85, 0.62, 0.10), "tin_small": (0.75, 0.20, 0.18)}
SPOT_COLOR = (0.30, 0.62, 0.35)


def _tin_z(level: int) -> float:
    return SPOT_TOP + level * TIN_H + TIN_H / 2 + 0.0005 * (level + 1)


OBJECTS = [
    *[Box(f"spot_{s}", pos=(x, y, TOP + SPOT_SIZE[2] / 2), size=SPOT_SIZE, color=SPOT_COLOR)
      for s, (x, y) in SPOTS.items()],
    *[Cylinder(n, pos=(*SPOTS["left"], _tin_z(i)), radius=TINS[n], height=TIN_H, mass=TIN_KG[n], color=TIN_COLOR[n])
      for i, n in enumerate(ORDER)],
]

CONTACT_M = 0.006  # a tin's bottom within 6 mm of a surface's top rests on it
CENTRE_M = 0.02  # a tin counts as on a spot / on the tin below when its centre is within 2 cm of it
AWAY_M = 0.03  # grippers at least 3 cm from every tin for full credit
MAX_TILT_DEG = 15.0


def _clear(mem):
    mem.update(failed=False, why="")


TRIAL = hhkit.Trial(on_clear=_clear)


def _bottom(s) -> float:
    return float(s.pos[2]) - TIN_H / 2


def _top(s) -> float:
    return float(s.pos[2]) + TIN_H / 2


def rests_on(objects, a: str, b: str) -> bool:
    """Tin a sits on tin b: a's bottom at b's top and a's centre over b."""
    sa, sb = objects[a], objects[b]
    return abs(_bottom(sa) - _top(sb)) <= CONTACT_M and math.dist(sa.pos[:2], sb.pos[:2]) < TINS[b]


def support_of(objects, a: str):
    """What tin a stands on: a spot name, "bench", another tin's name, or None (in the air)."""
    s = objects[a]
    for b in TINS:
        if b != a and rests_on(objects, a, b):
            return b
    for name, xy in SPOTS.items():
        if abs(_bottom(s) - SPOT_TOP) <= CONTACT_M and math.dist(s.pos[:2], xy) < SPOT_SIZE[0] / 2 + TINS[a]:
            return name
    if abs(_bottom(s) - TOP) <= CONTACT_M:
        return "bench"
    return None


def grounded(objects, a: str, seen=()) -> bool:
    sup = support_of(objects, a)
    if sup is None:
        return False
    if sup in TINS:
        return sup not in seen and grounded(objects, sup, (*seen, a))
    return True


def violation(objects) -> str:
    for a in TINS:
        for b in TINS:
            if a != b and TINS[a] > TINS[b] and rests_on(objects, a, b):
                return f"{a} on {b}"
    for a in TINS:
        if support_of(objects, a) == "bench":
            return f"{a} set down on the bench, off the marked spots"
    up = [a for a in TINS if not grounded(objects, a)]
    if len(up) >= 2:
        return "two tins off their supports at once: " + ", ".join(up)
    return ""


def _track(objects) -> None:
    if TRIAL.once() and not TRIAL.mem["failed"]:
        why = violation(objects)
        if why:
            TRIAL.mem.update(failed=True, why=why)


def _held(objects, name) -> bool:
    s = objects[name]
    r = TINS[name]
    for g in TRIAL.grippers():
        for p in (g["tcp"], *g["tips"]):
            if hhkit.point_cylinder(p, s.pos, r, TIN_H) < AWAY_M:
                return True
    return False


def stacked_count(objects) -> int:
    """Tins correctly built up on the right spot, from the bottom: largest on the spot, then medium, then small."""
    k = 0
    below = None
    for n in ORDER:
        s = objects[n]
        if hhkit.tilt_deg(s) > MAX_TILT_DEG:
            break
        if below is None:
            ok = support_of(objects, n) == "right" and math.dist(s.pos[:2], SPOTS["right"]) <= CENTRE_M
        else:
            ok = rests_on(objects, n, below) and math.dist(s.pos[:2], objects[below].pos[:2]) <= CENTRE_M
        if not ok:
            break
        k += 1
        below = n
    return k


def _ladder(objects) -> float:
    if TRIAL.mem["failed"]:
        return 0.0
    k = stacked_count(objects)
    if k == 3 and any(_held(objects, n) for n in TINS):
        k = 2
    return k / 3


def success(objects) -> bool:
    """All three tins on the right spot, largest at the bottom, grippers clear; no rule broken. Held 1 s."""
    _track(objects)
    return bool(_ladder(objects) == 1.0 and all(objects[n].resting(0.02, 1.0) for n in TINS))


def score(objects) -> float:
    """0, latched, once a bigger tin has rested on a smaller one, a tin has been set down on the bench off the
    marked spots, or two tins were off their supports at the same time. Otherwise 1/3 per tin built up on the right spot from the bottom; the full stack counts 1.0 only with both
    grippers 3 cm clear of the tins."""
    _track(objects)
    return _ladder(objects)


def distance(objects) -> float:
    _track(objects)
    return math.dist(objects["tin_large"].pos[:2], SPOTS["right"])


# -- scripted demo (left arm) ------------------------------------------------------------------------------------
# Seven single-tin moves (small to right, medium to middle, small onto medium, large to right, small to left,
# medium onto large, small onto medium). Fingers close across the tin 1.8 cm above its bottom, so the fingertips
# stay 5 mm clear of the wider tin below.
GRIP_ABOVE_BOTTOM = 0.018
# The tins travel about 3 cm above the tallest stack they pass (two tins: the third is in the gripper); the open fingers
# hover 3 mm above the full starting stack. The arm reaches about 11.5 cm above the bench over the far spot.
TRAVEL_Z = TOP + 0.11
MOVES = [("tin_small", "right"), ("tin_medium", "middle"), ("tin_small", "middle"), ("tin_large", "right"),
         ("tin_small", "left"), ("tin_medium", "right"), ("tin_small", "right")]


def _grip_z(level: int) -> float:
    return SPOT_TOP + level * TIN_H + GRIP_ABOVE_BOTTOM


BENCH_XY = (0.14, 0.05)  # the unsafe control's extra put-down place: the bench in front of the spots
PLACES = {**SPOTS, "bench": BENCH_XY}


def _grip_z_on(place, level):
    return _grip_z(level) - (SPOT_SIZE[2] if place == "bench" else 0.0)


def _move(src, src_level, dst, dst_level):
    (sx, sy), (dx, dy) = PLACES[src], PLACES[dst]
    gz, pz = _grip_z_on(src, src_level), _grip_z_on(dst, dst_level) + 0.004
    return [
        Waypoint(1.4, (sx, sy, TRAVEL_Z), FINGER_OPEN, f"over the {src} spot"),
        Waypoint(1.2, (sx, sy, gz), FINGER_OPEN, "down"),
        Waypoint(0.7, (sx, sy, gz), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.4, (dx, dy, TRAVEL_Z), FINGER_CLOSED, f"over the {dst} spot"),
        Waypoint(1.2, (dx, dy, pz), FINGER_CLOSED, "set down"),
        Waypoint(0.6, (dx, dy, pz), FINGER_OPEN, "let go"),
        Waypoint(0.8, (dx, dy, TRAVEL_Z), FINGER_OPEN, "up"),
    ]


def _plan(moves):
    stacks = {"left": list(ORDER), "middle": [], "right": [], "bench": []}
    ws = []
    for tin, dst in moves:
        src = next(s for s, st in stacks.items() if st and st[-1] == tin)
        ws += _move(src, len(stacks[src]) - 1, dst, len(stacks[dst]))
        stacks[dst].append(stacks[src].pop())
    return ws


def _finish():
    _x, y = SPOTS["right"]
    return [Waypoint(1.2, (0.18, y, TOP + 0.14), FINGER_OPEN, "back off and up"),
            Waypoint(1.2, (0.16, 0.20, TRAVEL_Z), FINGER_OPEN, "clear of the tins")]


BENCH_MOVES = [("tin_small", "bench"), ("tin_medium", "middle"), ("tin_large", "right"), ("tin_medium", "right"),
               ("tin_small", "right")]


def bench_demo():
    """The unsafe control (the audit's bypass): park the small tin on the bench as a fourth spot; five moves."""
    return {"left": [*_plan(BENCH_MOVES), *_finish()]}


def safe_demo():
    # The arm leaves the finished stack by backing off towards itself while rising: at the far spot it can only reach
    # 11.5 cm up, and a first version that swung straight across from there clipped the top tin off in the simulator.
    _x, y = SPOTS["right"]
    return {"left": [*_plan(MOVES), Waypoint(1.2, (0.18, y, TOP + 0.14), FINGER_OPEN, "back off and up"),
                     Waypoint(1.2, (0.16, 0.20, TRAVEL_Z), FINGER_OPEN, "clear of the tins")]}


def whole_stack_demo():
    """The naive control: lift the whole stack by the bottom tin and set it on the right spot."""
    (sx, sy), (dx, dy) = SPOTS["left"], SPOTS["right"]
    gz = _grip_z(0)
    return {"left": [
        Waypoint(1.4, (sx, sy, TRAVEL_Z), FINGER_OPEN, "over the stack"),
        Waypoint(1.2, (sx, sy, gz), FINGER_OPEN, "down to the bottom tin"),
        Waypoint(0.7, (sx, sy, gz), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (sx, sy, gz + 0.04), FINGER_CLOSED, "lift"),
        Waypoint(1.4, (dx, dy, gz + 0.04), FINGER_CLOSED, "carry"),
        Waypoint(1.0, (dx, dy, gz + 0.004), FINGER_CLOSED, "set down"),
        Waypoint(0.6, (dx, dy, gz + 0.004), FINGER_OPEN, "let go"),
        Waypoint(0.8, (dx, dy, TRAVEL_Z), FINGER_OPEN, "up"),
    ]}


PLANS = {"demo": safe_demo, "naive": whole_stack_demo, "unsafe": bench_demo}


def demo():
    """TASK_DEMO=naive or unsafe plays the negative controls."""
    return PLANS[hhkit.demo_variant()]()
