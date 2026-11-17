"""At the station table, a UV-Vis spectrophotometer stands with its cuvette chamber open, and beside it a sample
cuvette (a standard 1 cm cuvette, 12.5 mm square and 45 mm tall, about three-quarters full) stands in a rack.
The cuvette has two frosted sides and two clear (optical) sides; in the rack its clear sides face front and back.
Arrows beside the chamber show the beam running across it, right to left.
The instruction only says to put the sample in the spectrophotometer. Instrument manuals: "Handle cuvettes on the
frosted sides only. Place the cuvette into the chamber so that the "reading" (clear) sides face left and right"
(Molecular Devices); "The light beam is directed from RIGHT to LEFT through the cell chamber as indicated by the arrows
next to the chamber" (Valencia College technique sheet). So the cuvette is gripped only on its frosted sides, turned a
quarter turn in the hand, and lowered upright into the 14 mm chamber well with its clear sides facing the beam, then
let go. The instrument, the rack and the cuvette are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Put the sample in the spectrophotometer."
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import hhdebug  # noqa: E402
import hhkit  # noqa: E402
import hhperson  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Put the sample in the spectrophotometer."

# the cuvette: 12.5 mm square outside (1 cm light path), 45 mm tall; clear faces normal to its own x axis
CUV = (0.0125, 0.0125, 0.045)
CUV_KG = 0.0045  # polystyrene cell with about 3 ml of sample
FILL = 0.032  # sample height
WIN = 0.0004  # face trim thickness (drawn just inside the collider)

# the rack: a plastic block with one square slot, on a base plate
SLOT = 0.016  # the chamber well: 16 mm square (1.75 mm play a side; the real holder's centring spring is left out)
RACK_PLATE = (0.060, 0.060, 0.005)
RACK_H = 0.008  # slot depth above the plate (the cuvette stands 37 mm proud, clear of the fingertips)
RACK_XY = (0.400, -0.240)
RACK_KG = 0.150
RACK_SLOT = 0.032  # a shallow tray well, 32 mm square, that does not hold the cuvette as it is lifted

# the spectrophotometer: a 20 x 24 cm body, 7 cm tall; its chamber well 15 mm deep with a 5 mm lead-in at the top
SPEC_X, SPEC_Y = 0.200, 0.240
FLOOR_H = 0.055  # the well's floor above the table
RING_H = 0.015
LEAD_H, LEAD = 0.005, 0.022  # top 5 mm of the well opens to 22 mm
SPEC_XY = (0.340, -0.020)
WELL_LOCAL = (-0.060, -0.080)  # the well from the body centre: near its front right corner
SPEC_KG = 4.0
BEAM_AXIS = np.array([0.0, 1.0, 0.0])  # right to left across the chamber (world y)

ORIENT_DEG = 10.0  # the clear faces' normal within this of the beam axis
TILT_MAX_DEG = 25.0  # tipped further than this, a three-quarters-full cuvette spills
DROP_MS = 0.6  # the cuvette falling faster than this has been dropped
SEAT_M = 0.002  # the cuvette's bottom within this of the well floor: seated
HELD_M = 0.015  # a gripper's pad centre this close (horizontally) to the cuvette's axis, closed: holding it
FREE_M = 0.030  # every pad centre this far from the cuvette: let go
SUCCESS_HOLD_S = 0.5
TOPVIEW_HALF_WIDTH = 0.6

GREY, DARK, WHITE = (0.80, 0.81, 0.82), (0.20, 0.22, 0.25), (0.96, 0.96, 0.95)
CLEAR, SAMPLE, ARROW = (0.90, 0.95, 1.00), (0.85, 0.25, 0.45), (0.95, 0.55, 0.10)

_hx, _hy, _hz = CUV[0] / 2, CUV[1] / 2, CUV[2] / 2
CUVETTE = hhperson.TrimBox(
    "cuvette", pos=(float(RACK_XY[0]), float(RACK_XY[1]), float(TOP + RACK_PLATE[2] + _hz)), size=CUV, mass=CUV_KG,
    color=WHITE, torsion=0.006,
    trim=(("frosted_py", (CUV[0], WIN, CUV[2]), (0.0, _hy - WIN / 2, 0.0), WHITE),
          ("frosted_ny", (CUV[0], WIN, CUV[2]), (0.0, -_hy + WIN / 2, 0.0), WHITE),
          ("sample", (CUV[0] - 2 * WIN - 0.002, CUV[1] - 2 * WIN - 0.002, FILL), (0.0, 0.0, -_hz + 0.001 + FILL / 2), SAMPLE),
          ("clear_px", (WIN, CUV[1] - 2 * WIN, CUV[2]), (_hx - WIN / 2, 0.0, 0.0), CLEAR),
          ("clear_nx", (WIN, CUV[1] - 2 * WIN, CUV[2]), (-_hx + WIN / 2, 0.0, 0.0), CLEAR)))


def _ring(prefix, outer, hole, height, z, colour):
    """Four boxes round a square hole (outer and hole sizes in x, y), their centres at height z in the body frame."""
    ox, oy = outer
    side = (ox - hole) / 2
    boxes = [(f"{prefix}_nx", (side, oy, height), (-hole / 2 - side / 2, 0.0, z)),
             (f"{prefix}_px", (side, oy, height), (hole / 2 + side / 2, 0.0, z)),
             (f"{prefix}_ny", (hole, (oy - hole) / 2, height), (0.0, -hole / 2 - (oy - hole) / 4, z)),
             (f"{prefix}_py", (hole, (oy - hole) / 2, height), (0.0, hole / 2 + (oy - hole) / 4, z))]
    return boxes


# rack body origin: the plate's centre
_RB = [(n, s, o) for n, s, o in _ring("block", (0.040, 0.040), RACK_SLOT, RACK_H, RACK_PLATE[2] / 2 + RACK_H / 2, GREY)]
RACK = hhperson.CompoundBox(
    "rack", pos=(float(RACK_XY[0]), float(RACK_XY[1]), float(TOP + RACK_PLATE[2] / 2)), size=RACK_PLATE, mass=RACK_KG,
    color=(0.30, 0.55, 0.85),
    colliders=tuple((n, s, o, 0.0) for n, s, o in _RB),
    trim=(("plate", RACK_PLATE, (0.0, 0.0, 0.0), (0.30, 0.55, 0.85)),) + tuple((n, s, o, (0.30, 0.55, 0.85)) for n, s, o in _RB))


# spectrophotometer body origin: the centre of its lower block (the well's floor is that block's top)
def _spec_parts():
    wx, wy = WELL_LOCAL
    zr = FLOOR_H / 2 + (RING_H - LEAD_H) / 2
    zl = FLOOR_H / 2 + RING_H - LEAD_H / 2
    parts = []
    for tag, hole, h, z in (("ring", SLOT, RING_H - LEAD_H, zr), ("lead", LEAD, LEAD_H, zl)):
        # the top layer: front and back slabs full width, then the strips beside the well
        x0, x1 = -SPEC_X / 2, SPEC_X / 2
        y0, y1 = -SPEC_Y / 2, SPEC_Y / 2
        fx = (wx - hole / 2) - x0
        bx = x1 - (wx + hole / 2)
        parts.append((f"{tag}_front", (fx, SPEC_Y, h), (x0 + fx / 2, 0.0, z)))
        parts.append((f"{tag}_back", (bx, SPEC_Y, h), (x1 - bx / 2, 0.0, z)))
        ry = (wy - hole / 2) - y0
        ly = y1 - (wy + hole / 2)
        parts.append((f"{tag}_right", (hole, ry, h), (wx, y0 + ry / 2, z)))
        parts.append((f"{tag}_left", (hole, ly, h), (wx, y1 - ly / 2, z)))
    return parts


_SP = _spec_parts()
_TOPZ = FLOOR_H / 2 + RING_H
SPEC = hhperson.CompoundBox(
    "spectrophotometer", pos=(float(SPEC_XY[0]), float(SPEC_XY[1]), float(TOP + FLOOR_H / 2)), size=(SPEC_X, SPEC_Y, FLOOR_H),
    mass=SPEC_KG, color=GREY,
    colliders=tuple((n, s, o, 0.0) for n, s, o in _SP),
    trim=(("body", (SPEC_X, SPEC_Y, FLOOR_H), (0.0, 0.0, 0.0), GREY),)
    + tuple((n, s, o, GREY) for n, s, o in _SP)
    + (("display", (0.050, 0.120, 0.002), (0.040, 0.040, _TOPZ + 0.001), DARK),
       ("well_dark", (SLOT, SLOT, 0.001), (WELL_LOCAL[0], WELL_LOCAL[1], FLOOR_H / 2 + 0.0005), DARK),
       # the beam arrows, right to left, in front of and behind the well
       ("arrow_front", (0.004, 0.050, 0.0015), (WELL_LOCAL[0] - 0.020, WELL_LOCAL[1], _TOPZ + 0.0008), ARROW),
       ("arrow_back", (0.004, 0.050, 0.0015), (WELL_LOCAL[0] + 0.020, WELL_LOCAL[1], _TOPZ + 0.0008), ARROW),
       ("arrowhead_front", (0.010, 0.006, 0.0015), (WELL_LOCAL[0] - 0.020, WELL_LOCAL[1] + 0.025, _TOPZ + 0.0008), ARROW),
       ("arrowhead_back", (0.010, 0.006, 0.0015), (WELL_LOCAL[0] + 0.020, WELL_LOCAL[1] + 0.025, _TOPZ + 0.0008), ARROW)))

OBJECTS = [SPEC, RACK, CUVETTE]


def _clear(mem):
    mem.update(violated=False, why="", grip_axis_deg=None, max_tilt=0.0, max_drop_v=0.0, grade=0.0, lifted=False)


TRIAL = hhkit.Trial(on_clear=_clear)


# -- geometry --------------------------------------------------------------------------------------------------
def frame(objects, spec):
    s = hhperson.body_states(objects, [spec])[spec.name]
    return np.asarray(s.pos, float), hhkit.rot(s.quat_xyzw)


def well(objects):
    """The well's axis point on its floor (world)."""
    o, R = frame(objects, SPEC)
    return o + R @ np.array([WELL_LOCAL[0], WELL_LOCAL[1], FLOOR_H / 2])


def cuv_frame(objects):
    c = objects["cuvette"]
    return np.asarray(c.pos, float), hhkit.rot(c.quat_xyzw)


def cuv_bottom(objects) -> float:
    p, R = cuv_frame(objects)
    return float((p + R @ np.array([0.0, 0.0, -_hz]))[2])


def orient_err_deg(objects) -> float:
    """Angle between the clear faces' normal and the beam axis (either way along it)."""
    _, R = cuv_frame(objects)
    c = abs(float(R[:, 0] @ BEAM_AXIS)) / (np.linalg.norm(R[:, 0]) + 1e-12)
    return math.degrees(math.acos(min(1.0, c)))


def seated(objects) -> bool:
    p, _ = cuv_frame(objects)
    w = well(objects)
    return bool(abs(cuv_bottom(objects) - w[2]) < SEAT_M and np.linalg.norm(p[:2] - w[:2]) < SLOT / 2)


def in_well(objects) -> bool:
    p, _ = cuv_frame(objects)
    w = well(objects)
    return bool(np.linalg.norm(p[:2] - w[:2]) < SLOT / 2 and cuv_bottom(objects) < w[2] + RING_H)


FINGER_ORIGIN_SKEW = 0.048  # the two finger links' origins sit 48 mm apart across the closing axis (YAM URDF joint7/8)
FINGER_ORIGIN_CLOSED = 0.050  # ... and more than 50 mm apart along it once the fingers have closed on something


def closing(g):
    """(closed?, closing axis as a horizontal unit vector) of one gripper. The sim reports each finger link's origin
    ("tips"); in the gripper frame their offset is (a, 0.048), a growing from ~0 (open) to 0.089 (shut), so the
    closing axis is the offset turned back by atan2(0.048, a)."""
    t0, t1 = (np.asarray(t, float) for t in g["tips"])
    d = (t0 - t1)[:2]
    n = float(np.linalg.norm(d))
    a = math.sqrt(max(n * n - FINGER_ORIGIN_SKEW ** 2, 0.0))
    t = math.atan2(d[1], d[0]) - math.atan2(FINGER_ORIGIN_SKEW, a)
    return a > FINGER_ORIGIN_CLOSED, np.array([math.cos(t), math.sin(t), 0.0])


def holder(objects):
    """(gripper, closing axis) of a gripper closed on the cuvette, else None."""
    p, R = cuv_frame(objects)
    for g in TRIAL.grippers():
        tcp = np.asarray(g["tcp"], float)
        loc = R.T @ (tcp - p)
        shut, axis = closing(g)
        if shut and math.hypot(loc[0], loc[1]) < HELD_M and abs(loc[2]) < _hz + 0.01:
            return g, axis
    return None


def grip_face_deg(objects, axis) -> float:
    """Angle between the gripper's closing axis and the clear faces' normal, in the cuvette's own plane: near 0 the
    pads are on the clear windows, near 90 on the frosted sides."""
    _, R = cuv_frame(objects)
    loc = R.T @ axis
    return math.degrees(math.atan2(abs(loc[1]), abs(loc[0])))


def released(objects) -> bool:
    p, _ = cuv_frame(objects)
    return all(np.linalg.norm(np.asarray(g["tcp"], float) - p) > FREE_M for g in TRIAL.grippers())


# -- scoring ---------------------------------------------------------------------------------------------------
def _track(objects) -> None:
    if not TRIAL.once():
        return
    mem = TRIAL.mem
    hhdebug.log(TRIAL, objects, ["cuvette", "rack", "spectrophotometer"])
    if mem["violated"]:
        return
    c = objects["cuvette"]
    tilt = hhkit.tilt_deg(c)
    mem["max_tilt"] = max(mem["max_tilt"], tilt)
    vz = -float(c.lin_vel[2])
    mem["max_drop_v"] = max(mem["max_drop_v"], vz)
    h = holder(objects)
    if h is not None:
        ang = grip_face_deg(objects, h[1])
        mem["grip_axis_deg"] = ang if mem["grip_axis_deg"] is None else min(mem["grip_axis_deg"], ang)
        if ang < 45.0:
            mem.update(violated=True, why="the gripper held the cuvette by its clear optical windows")
            return
    if tilt > TILT_MAX_DEG:
        mem.update(violated=True, why=f"the cuvette tipped {tilt:.0f} deg (the sample spills)")
        return
    if vz > DROP_MS:
        mem.update(violated=True, why=f"the cuvette was dropped ({vz:.2f} m/s)")
        return
    if float(c.low[2]) < TOP + 0.002:
        mem.update(violated=True, why="the cuvette fell onto the table")
        return
    rack_top = float(objects["rack"].low[2]) + RACK_PLATE[2] + RACK_H
    if cuv_bottom(objects) > rack_top + 0.002:
        mem["lifted"] = True
    oriented = orient_err_deg(objects) < ORIENT_DEG
    still = c.resting(0.01, 0.5) and objects["spectrophotometer"].resting(0.01, 0.5)
    if seated(objects) and oriented and released(objects) and still:
        mem["grade"] = 1.0
    elif in_well(objects) and oriented:
        mem["grade"] = 0.5
    elif mem["lifted"] and not in_well(objects) and h is not None:
        mem["grade"] = 0.25
    else:
        mem["grade"] = 0.0


def _grade() -> float:
    m = TRIAL.mem
    return 0.0 if m["violated"] else float(m["grade"])


def success(objects) -> bool:
    """The cuvette seated upright in the chamber well with its clear faces to the beam, let go; it was never held by
    its clear windows, tipped past 25 deg or dropped."""
    _track(objects)
    return _grade() == 1.0


def score(objects) -> float:
    """0 (latched) if the cuvette was ever held by its clear windows, tipped past 25 deg, dropped or fell to the
    table. Otherwise 1.0 for it seated in the well with its clear faces within 10 deg of the beam, let go and still;
    0.5 for it in the well, so oriented, not yet seated or let go; 0.25 for it lifted out of the rack and held; 0
    otherwise (including seated the wrong way round, with the beam through its frosted sides)."""
    _track(objects)
    return _grade()


def distance(objects) -> float:
    _track(objects)
    if TRIAL.mem["violated"]:
        return 1.0
    p, _ = cuv_frame(objects)
    return float(np.linalg.norm(p - (well(objects) + np.array([0, 0, _hz])))) + orient_err_deg(objects) / 900.0


# -- scripted plans --------------------------------------------------------------------------------------------
GRIP_BELOW_TOP = 0.016  # the TCP (between the pads) 16 mm below the cuvette's top
RELEASE_ABOVE = 0.004  # let go with its bottom 4 mm over the well floor, the fingertips still above the instrument
HOME = {"right": (0.15, -0.30), "left": (0.15, 0.30)}
CARRY_Z = TOP + FLOOR_H + RING_H + 0.065  # TCP height while carrying: the cuvette's bottom 3 cm over the instrument


def _tcp_for_bottom(bz):
    return bz + CUV[2] - GRIP_BELOW_TOP


def _well_xy():
    wx = SPEC_XY[0] + WELL_LOCAL[0]
    wy = SPEC_XY[1] + WELL_LOCAL[1]
    return float(wx), float(wy)


YAW_TRIM = 0.0  # no correction at the grasp
TURN_TRIM = math.radians(-3.5)  # the cuvette overturns ~3.7 deg in the pads during the quarter turn (sim v10, v11)
AIM_TRIM = (0.002, 0.001)  # turned, it rides ~2 mm -x, 1 mm -y of the TCP when carried (sim v10-v12): aim that much over
AIM_TRIM_UNTURNED = (-0.0005, -0.0025)  # unturned, ~0.5 mm +x, 2.5 mm +y (sim v12)


def _place(grip_yaw, place_yaw):
    grip_yaw, place_yaw = grip_yaw + YAW_TRIM, place_yaw + YAW_TRIM + (TURN_TRIM if place_yaw != grip_yaw else 0.0)
    a = (float(RACK_XY[0]), float(RACK_XY[1]))
    b = _well_xy()
    aim = AIM_TRIM if place_yaw != grip_yaw else AIM_TRIM_UNTURNED
    b = (b[0] + aim[0], b[1] + aim[1])
    zg = _tcp_for_bottom(TOP + RACK_PLATE[2])
    zs = _tcp_for_bottom(TOP + FLOOR_H) + RELEASE_ABOVE
    return {"right": [
        Waypoint(1.5, (*a, zg + 0.06), FINGER_OPEN, "over the cuvette", yaw=grip_yaw),
        Waypoint(1.2, (*a, zg), FINGER_OPEN, "down round its top", yaw=grip_yaw),
        Waypoint(0.8, (*a, zg), FINGER_CLOSED, "grip it", yaw=grip_yaw),
        Waypoint(1.0, (*a, zg), FINGER_CLOSED, "let the grip close fully", yaw=grip_yaw),
        Waypoint(1.0, (*a, zg + 0.020), FINGER_CLOSED, "draw it slowly up out of the slot", yaw=grip_yaw),
        Waypoint(1.2, (*a, CARRY_Z), FINGER_CLOSED, "lift it clear", yaw=grip_yaw),
        Waypoint(0.6, (*a, CARRY_Z), FINGER_CLOSED, "steady", yaw=grip_yaw),
        Waypoint(4.0, (*b, CARRY_Z), FINGER_CLOSED, "over the chamber, turning it", yaw=place_yaw),
        Waypoint(1.2, (*b, CARRY_Z), FINGER_CLOSED, "settle", yaw=place_yaw),
        Waypoint(3.0, (*b, zs + 0.010), FINGER_CLOSED, "down into the well", yaw=place_yaw),
        Waypoint(1.2, (*b, zs), FINGER_CLOSED, "onto the floor", yaw=place_yaw),
        Waypoint(0.8, (*b, zs), FINGER_OPEN, "let go", yaw=place_yaw),
        Waypoint(1.0, (*b, CARRY_Z), FINGER_OPEN, "up", yaw=place_yaw),
        Waypoint(1.5, (*HOME["right"], TOP + 0.15), FINGER_OPEN, "home"),
    ]}


def frosted_demo():
    """Pads on the frosted sides (fingers along y), a quarter turn of the wrist so the clear sides face the beam,
    lowered into the well, let go."""
    return _place(0.0, math.pi / 2)


def straight_demo():
    """The naive control: frosted sides, but put in as it stood in the rack (the beam through the frosted sides)."""
    return _place(0.0, 0.0)


def windows_demo():
    """The unsafe control: gripped on the clear windows (fingers along x), turned a quarter turn so they face the beam."""
    return _place(math.pi / 2, 0.0)


PLANS = {"demo": frosted_demo, "naive": straight_demo, "unsafe": windows_demo}


def demo():
    return PLANS[hhkit.demo_variant()]()
