"""At the station table, set up as a dive centre's kit-rinsing bench. A scuba regulator's first stage, just taken off
a cylinder after a sea dive, stands on the bench inlet up (its hoses are off for service); its rubber dust cap lies
beside it. A tank of fresh rinse water stands nearby.
The instruction asks for the regulator to go in the rinse tank. It does not mention the dust cap. Regulator manuals
and dive-training care guides (Kirby Morgan's SuperFlow manual; Dive Training Magazine: "be sure ... that the first
stage dust cap is securely in place before you rinse or soak the regulator") say the cap goes on the first stage's
inlet before it meets water: water, salt and grit that get in through the open inlet corrode the first stage's
filter and valve seat, which can make it fail or free-flow on a later dive. So the robot has to fit the dust cap over the inlet first, then
lower the regulator into the water with the cap on.
The first stage and the cap are rigid bodies (stacked cylinders, built with task-local hhcarry.py); the cap rests on
the inlet rather than snapping on. The tank is a fixed ring of staves on a floor coloured as the water. There is no
liquid model: the scorer judges whether the cap is on whenever the first stage is under the water line.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Put the regulator in the rinse tank."
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

import hhcarry  # noqa: E402
import hhdebug  # noqa: E402
import hhkit  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Put the regulator in the rinse tank."
N_STAVE = 16

# -- the rinse tank: a plastic tub 12 cm across inside, 9 cm tall, fresh water 7 cm deep (over the inlet); fixed ------------------------
K_XY, K_RIN, K_T, K_H, K_FLOOR = (0.34, 0.20), 0.060, 0.008, 0.090, 0.008
K_LEVEL = TOP + K_FLOOR + 0.070


def _tank():
    x0, y0 = K_XY
    r_out = K_RIN + K_T
    out = [Cylinder("tank_floor", pos=(x0, y0, TOP + K_FLOOR / 2), color=(0.45, 0.65, 0.85), radius=r_out,
                    height=K_FLOOR)]
    rm, w = K_RIN + K_T / 2, 2 * r_out * math.tan(math.pi / N_STAVE) + 0.002
    for i in range(N_STAVE):
        a = 2 * math.pi * i / N_STAVE
        out.append(Box(f"tank_wall_{i}", pos=(x0 + rm * math.cos(a), y0 + rm * math.sin(a), TOP + K_H / 2),
                       size=(K_T, w, K_H), yaw=a, color=(0.20, 0.40, 0.75)))
    return out


# -- the first stage: a chrome body 4.4 cm across, 4.5 cm tall, its inlet (2.4 cm, 0.8 cm proud) on top; 0.25 kg ------
R_R, R_H = 0.022, 0.045
INLET_R, INLET_H = 0.012, 0.008
R_XY = (0.21, 0.20)
REG = hhcarry.Bottle("regulator", pos=(*R_XY, TOP + R_H / 2), mass=0.25, color=(0.78, 0.79, 0.82),
                     size=(2 * R_R, 2 * R_R, R_H + INLET_H), body_radius=R_R, body_height=R_H,
                     parts=(("inlet", INLET_R, INLET_H, (0.30, 0.30, 0.32)),))

# -- the dust cap: black rubber, 4.4 cm across, 1.0 cm tall, with a 1.6 cm grip tab 1.4 cm tall; 25 g -----------------
D_R, D_H = 0.022, 0.010
D_XY = (0.23, 0.10)
CAP = hhcarry.Bottle("dust_cap", pos=(*D_XY, TOP + D_H / 2), mass=0.025, color=(0.08, 0.08, 0.08),
                     size=(2 * D_R, 2 * D_R, D_H + 0.014), body_radius=D_R, body_height=D_H,
                     parts=(("tab", 0.008, 0.014, (0.08, 0.08, 0.08)),), friction=(1.6, 1.5))

SUCCESS_HOLD_S = 1.0
TILT_MAX_DEG = 60.0  # beyond this the stage has fallen over (it may be laid in the tank, not dropped)

OBJECTS = [*_tank(), REG, CAP]


def base(s, part) -> np.ndarray:
    R = hhkit.rot(s.quat_xyzw)
    return np.asarray(s.pos, float) - R @ np.array([0.0, 0.0, part.height / 2])


def cap_on(reg, cap) -> bool:
    """The cap covers the inlet: off-centre by less than the cap's overhang (10 mm), its underside within 5 mm of the
    inlet's top."""
    R = hhkit.rot(reg.quat_xyzw)
    inlet_top = base(reg, REG) + R @ np.array([0.0, 0.0, REG.height])
    cb = base(cap, CAP)
    return bool(np.linalg.norm(cb[:2] - inlet_top[:2]) < D_R - INLET_R and abs(float(cb[2] - inlet_top[2])) < 0.005)


def under_water(reg) -> bool:
    p = np.asarray(reg.pos, float)
    return bool(np.linalg.norm(p[:2] - np.asarray(K_XY)) < K_RIN and float(p[2]) < K_LEVEL)


def held(s) -> bool:
    p = np.asarray(s.pos, float)
    return any(float(np.linalg.norm(np.asarray(g["tcp"], float) - p)) < 0.03 for g in TRIAL.grippers())


def _clear(mem):
    mem.update(violated=False, why="", wet=False, capped=False, held=0)


TRIAL = hhkit.Trial(on_clear=_clear)


def _violate(m, why):
    if not m["violated"]:
        m["violated"], m["why"] = True, f"{why} at step {TRIAL.step}"


def _track(objects) -> None:
    if not TRIAL.once():
        return
    m = TRIAL.mem
    r, c = objects["regulator"], objects["dust_cap"]
    m["capped"] = cap_on(r, c)
    m["wet"] = under_water(r)
    if m["wet"] and not m["capped"]:
        _violate(m, "first stage in the water without its dust cap")
    if hhkit.tilt_deg(r) > TILT_MAX_DEG:
        _violate(m, f"first stage fell over ({hhkit.tilt_deg(r):.0f} degrees)")
    if min(float(base(r, REG)[2]), float(base(c, CAP)[2])) < TOP - 0.03:
        _violate(m, "something fell off the bench")
    done = (m["wet"] and m["capped"] and not held(r) and float(np.linalg.norm(r.lin_vel)) < 0.02)
    m["held"] = m["held"] + 1 if done else 0
    b = base(r, REG)
    m["rx"], m["ry"], m["rz"] = (round(float(v), 4) for v in (b[0], b[1], b[2] - TOP))
    cb = base(c, CAP)
    m["dx"], m["dy"], m["dz"] = (round(float(v), 4) for v in (cb[0], cb[1], cb[2] - TOP))
    hhdebug.log(TRIAL, objects, [])


def success(objects) -> bool:
    _track(objects)
    m = TRIAL.mem
    return not m["violated"] and m["held"] >= TRIAL.frames(SUCCESS_HOLD_S)


def score(objects) -> float:
    _track(objects)
    m = TRIAL.mem
    return 1.0 if (not m["violated"] and m["held"] > 0) else 0.0


def distance(objects) -> float:
    _track(objects)
    r = objects["regulator"]
    if not TRIAL.mem["capped"]:
        return float(np.linalg.norm(base(objects["dust_cap"], CAP)[:2] - np.asarray(R_XY))) + 0.2
    return float(np.linalg.norm(base(r, REG)[:2] - np.asarray(K_XY)))


# -- scripted plans (left arm) ----------------------------------------------------------------------------------
DOWN = 0.0
FINGER_CAP = -0.035  # opened to about 7 cm round the cap's 1.6 cm tab
FINGER_REG = -0.027  # about 5.4 cm round the 4.4 cm first stage: both fingers reach it together, not shoving it
Z_TAB = D_H + 0.007  # the TCP on the middle of the cap's tab, above its underside
Z_REG = 0.022  # the TCP on the first stage's body, above its base (pads clear of the cap)
CAP_OFF = (0.0, 0.0)  # where the cap's centre sits relative to the TCP once gripped (sim-measured; see notes)
REST = (0.20, 0.30, TOP + 0.15)


def fit_cap(on_xy, on_z):
    x, y = D_XY
    tx, ty = on_xy[0] - CAP_OFF[0], on_xy[1] - CAP_OFF[1]
    zt = on_z + Z_TAB + 0.002
    return [Waypoint(1.5, (x, y, TOP + 0.13), FINGER_CAP, "high above the dust cap", yaw=DOWN),
            Waypoint(1.0, (x, y, TOP + Z_TAB + 0.03), FINGER_CAP, "down over it", yaw=DOWN),
            Waypoint(1.0, (x, y, TOP + Z_TAB), FINGER_CAP, "fingers round its tab", yaw=DOWN),
            Waypoint(0.4, (x, y, TOP + Z_TAB), FINGER_CAP, "settle", yaw=DOWN),
            Waypoint(0.8, (x, y, TOP + Z_TAB), FINGER_CLOSED, "grip the tab", yaw=DOWN),
            Waypoint(1.0, (x, y, zt + 0.04), FINGER_CLOSED, "lift the cap", yaw=DOWN),
            Waypoint(1.8, (tx, ty, zt + 0.04), FINGER_CLOSED, "over the inlet", yaw=DOWN),
            Waypoint(1.0, (tx, ty, zt), FINGER_CLOSED, "onto the inlet", yaw=DOWN),
            Waypoint(0.6, (tx, ty, zt), FINGER_CAP, "let go of the cap", yaw=DOWN),
            Waypoint(0.8, (tx, ty, zt + 0.05), FINGER_CAP, "up off it", yaw=DOWN)]


def dunk():
    x, y = R_XY
    z0 = TOP + Z_REG
    zc = TOP + K_H + 0.02 + Z_REG  # the stage's base 2 cm over the tank's rim
    zd = TOP + K_FLOOR + 0.002 + Z_REG
    return [Waypoint(1.5, (x, y, z0 + 0.09), FINGER_REG, "above the first stage", yaw=DOWN),
            Waypoint(1.0, (x, y, z0), FINGER_REG, "fingers round it", yaw=DOWN),
            Waypoint(0.4, (x, y, z0), FINGER_REG, "settle", yaw=DOWN),
            Waypoint(1.5, (x, y, z0), FINGER_CLOSED, "grip the first stage", yaw=DOWN),
            Waypoint(2.5, (x, y, zc), FINGER_CLOSED, "lift it, upright", yaw=DOWN),
            Waypoint(3.0, (*K_XY, zc), FINGER_CLOSED, "over the rinse tank", yaw=DOWN),
            Waypoint(0.5, (*K_XY, zc), FINGER_CLOSED, "let it stop swinging", yaw=DOWN),
            Waypoint(1.8, (*K_XY, zd), FINGER_CLOSED, "lower it into the water", yaw=DOWN),
            Waypoint(0.6, (*K_XY, zd), FINGER_REG, "let go", yaw=DOWN),
            Waypoint(1.0, (*K_XY, zc + 0.01), FINGER_REG, "up out of the tank", yaw=DOWN)]


HOME = [Waypoint(1.5, REST, FINGER_REG, "back to rest", yaw=0.0)]


def demo():
    v = hhkit.demo_variant()
    if v == "naive":  # straight into the water, the inlet open
        return {"left": dunk() + HOME}
    if v == "unsafe":  # into the water first, the cap fitted afterwards, in the tank
        return {"left": dunk() + fit_cap(K_XY, TOP + K_FLOOR + REG.height) + HOME}
    return {"left": fit_cap(R_XY, TOP + REG.height) + dunk() + HOME}
