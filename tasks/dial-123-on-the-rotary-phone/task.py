"""At the station table, a desk telephone with a rotary dial, of the kind still sold new as reproductions of the GPO 746
(the British Post Office's standard phone from 1959). Its handset is already off the hook, lying on the desk beside it.
The instruction asks for 123 to be dialled (the UK speaking clock's number).

Dialling a rotary phone takes a particular movement that anyone who has used one knows: put one finger into the hole
for the digit, turn the finger wheel clockwise until the finger meets the metal finger stop, then take the finger out
and let the wheel run back by itself. The phone sends the digit while the wheel runs back, as one break in the line
per 30 degrees of turn, timed by the dial's governor, so (as on a real dial) the digit is set by how far the wheel was
turned, not by which hole was used: stopping short of the finger stop sends a smaller digit, or none; riding the
wheel back with the finger upsets the pulse timing and garbles the digit. Either is a wrong number, and the caller has
to hang up and start again.

The finger wheel is a loose body on a sprung, damped hinge (task-local bodykit.py) above a fixed number ring; its ten
holes are the gaps between eleven spokes, the finger stop is a rigid part of the phone, and the phone is a loose
1.0 kg body on rubber feet. The scorer reads the wheel's turn every control step and decodes the digits as above.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Dial 123 on the phone."
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bodykit import Body, BoxShape, CylShape, as_numpy, hinge, on_reset, on_step, rotation, segment  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Dial 123 on the phone."
NUMBER = "123"
HZ = 20.0

IVORY, CREAM, BLACK, WHITE = (0.86, 0.82, 0.70), (0.80, 0.76, 0.64), (0.04, 0.04, 0.05), (0.95, 0.95, 0.93)
STEEL, GREY, RED = (0.70, 0.71, 0.73), (0.35, 0.35, 0.37), (0.75, 0.10, 0.10)

# -- the phone (a GPO 746 is 21.5 cm wide, 16 cm deep, 1.0 kg): a base 6 cm tall, the dial in front, the cradle behind
P_XY, P_SIZE = (0.30, 0.16), (0.16, 0.215, 0.06)
P_TOP = TOP + P_SIZE[2]
D_XY = (0.272, 0.16)  # the dial's centre
RING_T = 0.002  # the white number ring on the base's top, under the wheel
W_T, W_GAP = 0.004, 0.001  # the finger wheel's thickness, and its gap above the number ring
W_BOT = P_TOP + RING_T + W_GAP
W_TOP = W_BOT + W_T
R_WHEEL, R_HUB = 0.050, 0.024  # the finger wheel is 10 cm across; the holes run out from the hub
R_PITCH = 0.036  # the holes' centres
SPOKE_W = 0.003
STOP_PSI = 120.0  # the finger stop, in degrees clockwise (seen from above) from straight ahead (+x)


FINGER_LEAD = 15.0  # degrees: a finger pressed against a hole's leading spoke meets the stop this much before the
# hole's centre would, so the holes sit this much further back for the turn to the stop to be 30 degrees per digit + 30


def hole_psi(digit: int) -> float:
    """Where a digit's hole sits on the wheel at rest, in degrees clockwise from +x: for 1 the finger is turned 60
    degrees to the finger stop, for each next digit 30 degrees more, for 0 (last) 330 degrees."""
    n = 10 if digit == 0 else digit
    return STOP_PSI - FINGER_LEAD - 30.0 * (n + 1)


def turn_for(digit: int) -> float:
    """How far (degrees) the wheel turns, finger in that digit's hole, until the finger meets the stop."""
    return 30.0 * ((10 if digit == 0 else digit) + 1)


def pulses(max_turn_deg: float) -> int:
    """Line breaks sent as the wheel runs back from ``max_turn_deg``: one per 30 degrees past the first 30, a part
    step not counting (a finger stopped 10 degrees short of the stop for 1 sends nothing)."""
    return max(0, int(math.floor((max_turn_deg - 15.0) / 30.0)))


def _dir(psi_deg: float, r: float) -> np.ndarray:
    a = math.radians(psi_deg)
    return np.array([r * math.cos(a), -r * math.sin(a)])


def _stop_shapes():
    """The finger stop's tongue and post (phone frame), built separately so segment() gets a real thickness."""
    dx, dy = D_XY[0] - P_XY[0], D_XY[1] - P_XY[1]
    sz = P_SIZE[2]
    s0, s1 = _dir(STOP_PSI, R_HUB + 0.004), _dir(STOP_PSI, R_WHEEL + 0.008)
    zc = W_TOP + 0.0015 + 0.004 - TOP
    a = -math.radians(STOP_PSI)
    mid = (s0 + s1) / 2
    length = float(np.linalg.norm(s1 - s0))
    return [BoxShape((dx + mid[0], dy + mid[1], zc), (length, 0.004, 0.008), STEEL, yaw=a),
            BoxShape((dx + s1[0], dy + s1[1], (sz + zc + 0.004) / 2), (0.006, 0.006, zc + 0.004 - sz), STEEL)]


SEG = {0: "abcdef", 1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc", 5: "afgcd", 6: "afgedc", 7: "abc", 8: "abcdefg",
       9: "abcdfg"}


def _digit(d: int, cx: float, cy: float, h: float, z: float) -> list:
    """A seven-segment digit ``h`` tall centred at (cx, cy), upright as the robot sees it (its top toward +x)."""
    w, t = 0.55 * h, 0.16 * h
    s = {"a": (h / 2, 0, w, t), "g": (0, 0, w, t), "d": (-h / 2, 0, w, t),
         "f": (h / 4, w / 2, t, h / 2), "b": (h / 4, -w / 2, t, h / 2),
         "e": (-h / 4, w / 2, t, h / 2), "c": (-h / 4, -w / 2, t, h / 2)}
    out = []
    for k in SEG[d]:
        ddx, ddy, sy, sx = s[k]
        out.append(BoxShape((cx + ddx, cy + ddy, z), (sx if k in "fbec" else t, sy if k in "agd" else t, 0.0004),
                            BLACK, collide=False))
    return out


def _phone_body():
    sx, sy, sz = P_SIZE
    dx, dy = D_XY[0] - P_XY[0], D_XY[1] - P_XY[1]
    shapes = [BoxShape((0.0, 0.0, sz / 2), P_SIZE, IVORY),
              BoxShape((0.0575, 0.0, sz + 0.012), (0.045, 0.20, 0.024), IVORY),  # the raised back with the cradle
              *[BoxShape((0.0575, s * 0.075, sz + 0.030), (0.045, 0.018, 0.012), CREAM) for s in (-1, 1)],
              *[BoxShape((0.0575, s * 0.075, sz + 0.040), (0.010, 0.010, 0.010), WHITE, collide=False)
                for s in (-1, 1)],  # the hook-switch plungers, up: off the hook
              CylShape((dx, dy, sz + RING_T / 2), R_WHEEL + 0.006, RING_T, WHITE)]  # the number ring
    for d in range(10):  # each digit printed on the ring, seen through its hole at rest (upright to the robot)
        c = _dir(hole_psi(d), R_PITCH)
        shapes += _digit(d, dx + c[0], dy + c[1], 0.009, sz + RING_T + 0.0002)
    shapes += _stop_shapes()
    return Body.build("phone", pos=(*P_XY, TOP), mass=1.0, shapes=shapes, color=IVORY, friction=(0.9, 0.8),
                      com=(0.0, 0.0, 0.03))


def _wheel():
    """The black finger wheel: a hub with a red centre label, an outer rim, and spokes between the ten holes; the 90
    degrees round the finger stop (between holes 1 and 0) is solid."""
    shapes = [CylShape((0.0, 0.0, 0.0), R_HUB, W_T, BLACK),
              CylShape((0.0, 0.0, W_T / 2 + 0.0003), 0.016, 0.0006, RED, collide=False)]
    rim_w = 0.004
    for k in range(36):  # the rim, 36 short boxes
        c = _dir(10.0 * k, R_WHEEL - rim_w / 2)
        shapes.append(BoxShape((c[0], c[1], 0.0), (rim_w, 2 * math.pi * R_WHEEL / 36 + 0.001, W_T), BLACK,
                               yaw=-math.radians(10.0 * k)))
    spoke_len = R_WHEEL - rim_w - R_HUB + 0.002
    rm = R_HUB + spoke_len / 2 - 0.001
    for k in range(11):  # spokes between the holes: from 1's leading side (60) round to 0's trailing side (-240)
        psi = hole_psi(1) + 15.0 - 30.0 * k
        c = _dir(psi, rm)
        shapes.append(BoxShape((c[0], c[1], 0.0), (spoke_len, SPOKE_W, W_T), BLACK, yaw=-math.radians(psi)))
    for psi in (67.5, 82.5, 97.5, 112.5):  # the solid part, 60 to 120 degrees, under the finger stop
        c = _dir(psi, rm)
        chord = 2 * (R_WHEEL - rim_w) * math.sin(math.radians(7.5)) + 0.001
        shapes.append(BoxShape((c[0], c[1], 0.0), (spoke_len, chord, W_T), BLACK, yaw=-math.radians(psi)))
    return Body.build("finger_wheel", pos=(*D_XY, W_BOT + W_T / 2), mass=0.04, shapes=shapes, color=BLACK,
                      ignore=("phone",), friction=(0.9, 0.8))


def _handset():
    """The handset, off the hook, lying on the desk to the phone's right; its curly cord is visual only."""
    shapes = [BoxShape((0.0, 0.0, 0.030), (0.12, 0.026, 0.020), IVORY),  # the handle, arched over the desk
              *[BoxShape((s * 0.062, 0.0, 0.026), (0.03, 0.03, 0.024), IVORY) for s in (-1, 1)],
              CylShape((-0.075, 0.0, 0.013), 0.026, 0.026, IVORY),  # the earpiece, face down
              CylShape((0.075, 0.0, 0.013), 0.024, 0.026, IVORY)]  # the mouthpiece, face down
    p0 = np.array([0.0, 0.03, 0.006])
    for k in range(6):  # the cord: a zig-zag to the phone's side
        p1 = np.array([-0.01 + 0.012 * (k % 2), 0.03 + 0.012 * (k + 1), 0.006])
        shapes.append(segment(p0, p1, 0.004, CREAM, collide=False))
        p0 = p1
    return Body.build("handset", pos=(0.34, -0.03, TOP), mass=0.5, shapes=shapes, kinematic=True, color=IVORY)


PHONE, WHEEL, HANDSET = _phone_body(), _wheel(), _handset()
OBJECTS = [PHONE, WHEEL, HANDSET]
# The dial's pre-wound spring and its governor, as one velocity drive: free, the wheel runs back at 300 degrees/s (10
# pulses a second, 30 degrees a pulse); held or turned, it pushes back with DIAL_DAMPING x (300 + turning speed),
# about 0.03 N m (some 0.8 N at the holes); at rest it is held against the 0 limit, as the real stop screw does.
RUN_BACK_DPS = 300.0
DIAL_DAMPING = 1.0e-4  # N m per degree/s
hinge("dial", "phone", "finger_wheel", anchor=(*D_XY, W_BOT + W_T / 2), axis="Z", limits_deg=(-340.0, 0.0),
      damping=DIAL_DAMPING, target_vel_deg=RUN_BACK_DPS)

START_DEG = 10.0  # a turn starts once the wheel is this far round
HOME_DEG = 4.0  # ... and has ended (the digit sent) once back within this of rest
FINGER_IN_Z = W_TOP + 0.001  # a pad tip below this (in a hole: below the wheel's top face) over the wheel
RIDE_FRAMES = 6  # a finger in a hole while the wheel runs back this many control steps (0.3 s) garbles the digit
CLEAR_Z = W_TOP + 0.03  # let go: every pad tip at least 3 cm above the wheel, or off it
MOVED_M = 0.02  # the phone pushed this far across the desk
SUCCESS_HOLD_S = 1.0
PAD = 0.013  # the pads reach this far below the TCP


def _fresh() -> dict:
    return {"turn": 0.0, "last_yaw": None, "in_turn": False, "max_turn": 0.0, "ride": 0, "garbled": False,
            "digits": "", "cum": 0.0, "violated": False, "why": "", "held": 0, "step": 0, "tcps": None, "phone": None}


EP = _fresh()


@on_reset
def _new_episode() -> None:
    EP.clear()
    EP.update(_fresh())


def _violate(why: str) -> None:
    if not EP["violated"]:
        EP["violated"], EP["why"] = True, f"{why} at step {EP['step']}"


def _finger_in_hole(tcps, centre_xy) -> bool:
    if tcps is None:
        return False
    for t in tcps:
        tip = np.asarray(t, float) - np.array([0.0, 0.0, PAD])
        r = float(np.linalg.norm(tip[:2] - np.asarray(centre_xy)))
        if tip[2] < FINGER_IN_Z and R_HUB - 0.008 < r < R_WHEEL + 0.004:
            return True
    return False


def _let_go(tcps, centre_xy) -> bool:
    if tcps is None:
        return True
    for t in tcps:
        tip = np.asarray(t, float) - np.array([0.0, 0.0, PAD])
        r = float(np.linalg.norm(tip[:2] - np.asarray(centre_xy)))
        if tip[2] < CLEAR_Z and r < R_WHEEL + 0.03:
            return False
    return True


def track(turn_deg, phone_xy, tcps) -> None:
    """Called once per control step with how far (degrees, clockwise from rest) the wheel is turned, where the phone
    is and the two TCPs."""
    EP["step"] += 1
    EP["tcps"] = tcps
    turn = float(turn_deg)
    prev = EP["turn"]
    EP["turn"] = turn
    centre = np.asarray(D_XY) + (np.asarray(phone_xy, float) - np.asarray(P_XY))
    if float(np.linalg.norm(np.asarray(phone_xy, float) - np.asarray(P_XY))) > MOVED_M:
        _violate("phone pushed across the desk")
    if not EP["in_turn"] and turn > START_DEG:
        EP.update(in_turn=True, max_turn=turn, ride=0)
    if EP["in_turn"]:
        EP["max_turn"] = max(EP["max_turn"], turn)
        if turn < prev - 0.5 and _finger_in_hole(tcps, centre):  # running back with a finger still in it
            EP["ride"] += 1
        if turn < HOME_DEG:
            n = pulses(EP["max_turn"])
            if n > 0:
                EP["digits"] += str(n % 10)
                if EP["ride"] >= RIDE_FRAMES:
                    EP["garbled"] = True
                    _violate(f"digit {n % 10} ridden back with the finger ({EP['ride']} steps)")
            EP["in_turn"] = False
    if EP["digits"] and not NUMBER.startswith(EP["digits"]):
        _violate(f"wrong number dialled: {EP['digits']}")
    done = (EP["digits"] == NUMBER and not EP["in_turn"] and turn < HOME_DEG and _let_go(tcps, centre))
    EP["held"] = EP["held"] + 1 if done else 0
    EP["phone"] = [round(float(v), 4) for v in phone_xy]


def _tcps(base, origin) -> list:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _rel_yaw_deg(r_phone, r_wheel) -> float:
    rel = r_phone.T @ r_wheel
    return math.degrees(math.atan2(rel[1, 0], rel[0, 0]))


_DEBUG = os.environ.get("HH_DEBUG")


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pp = as_numpy(base.scene["phone"].data.root_link_pose_w)[0]
    wp = as_numpy(base.scene["finger_wheel"].data.root_link_pose_w)[0]
    yaw = _rel_yaw_deg(rotation(pp[3:7]), rotation(wp[3:7]))  # counter-clockwise positive; a dial turns clockwise
    if EP["last_yaw"] is None:
        cum = -yaw
    else:
        d = yaw - EP["last_yaw"]
        d = (d + 180.0) % 360.0 - 180.0
        cum = EP["cum"] - d
    EP["last_yaw"], EP["cum"] = yaw, cum
    track(cum, pp[:2] - origin[:2], _tcps(base, origin))
    if _DEBUG and EP["step"] % 5 == 0:
        import json
        rec = {"step": EP["step"], "grippers": [{"tcp": [round(float(v), 4) for v in t]} for t in EP["tcps"]],
               "mem": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in EP.items()
                       if isinstance(v, (int, float, str, bool, list)) and k != "tcps"}}
        with open(_DEBUG, "a") as f:
            f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    return not EP["violated"] and EP["held"] >= int(SUCCESS_HOLD_S * HZ)


PER_DIGIT = 0.25  # partial credit: each correct digit sent, in order, while nothing has gone wrong


def score(objects) -> float:
    if EP["violated"]:
        return 0.0
    if EP["held"] >= int(SUCCESS_HOLD_S * HZ):
        return 1.0
    return PER_DIGIT * len(EP["digits"])  # 0.25 / 0.5 / 0.75 for 1 / 12 / 123 sent but not finished


def distance(objects) -> float:
    k = len(EP["digits"])
    if k >= len(NUMBER):
        return 0.0
    return float(len(NUMBER) - k) * 0.1


# -- scripted plans (left arm): the closed fingers as one finger, its 15 mm-wide tip turned to lie along the hole ----
REST = (0.20, 0.30, TOP + 0.20)
TIP_IN = W_BOT + 0.0005  # the pad tips 0.5 mm above the number ring: 3.5 mm down past the wheel's top
ABOVE = W_TOP + 0.04


def _at(psi_deg: float, z_tip: float) -> tuple:
    c = np.asarray(D_XY) + _dir(psi_deg, R_PITCH)
    return (float(c[0]), float(c[1]), z_tip + PAD)


def _yaw(psi_deg: float) -> float:
    return -math.radians(psi_deg)  # the closed tip's 15 mm width along the wheel's radius, its 8 mm across the hole


def dial(digit: int, short_deg: float = 0.0, ride: bool = False) -> list:
    """One digit: finger into its hole, round to the stop (or ``short_deg`` short of it), out, and wait for the wheel
    to run back; ``ride`` follows the wheel back down instead of taking the finger out."""
    p0 = hole_psi(digit)
    p1 = STOP_PSI - 3.0 - short_deg  # aim past where the finger meets the stop (~9.5 degrees short): the stop halts it
    out = [Waypoint(1.5, _at(p0, ABOVE), FINGER_CLOSED, f"above the {digit} hole", yaw=_yaw(p0)),
           Waypoint(1.0, _at(p0, TIP_IN), FINGER_CLOSED, f"finger into the {digit} hole", yaw=_yaw(p0))]
    steps = max(2, int(math.ceil((p1 - p0) / 10.0)))
    for k in range(1, steps + 1):
        p = p0 + (p1 - p0) * k / steps
        out.append(Waypoint(0.25, _at(p, TIP_IN), FINGER_CLOSED, "turn the wheel round" if k < steps else
                            "to the finger stop", yaw=_yaw(p)))
    out.append(Waypoint(0.4, _at(p1, TIP_IN), FINGER_CLOSED, "hold at the stop", yaw=_yaw(p1)))
    if ride:
        for k in range(1, steps + 1):
            p = p1 - (p1 - p0) * k / steps
            out.append(Waypoint(0.25, _at(p, TIP_IN), FINGER_CLOSED, "ride the wheel back", yaw=_yaw(p)))
        out.append(Waypoint(0.6, _at(p0, ABOVE), FINGER_CLOSED, "finger out", yaw=_yaw(p0)))
    else:
        out.append(Waypoint(0.3, _at(p1, ABOVE), FINGER_CLOSED, "finger straight out: the wheel runs back",
                            yaw=_yaw(p1)))
        out.append(Waypoint(0.8, _at(p1, ABOVE), FINGER_CLOSED, "wait for it to stop", yaw=_yaw(p1)))
    return out


HOME = [Waypoint(1.5, REST, FINGER_CLOSED, "back to rest", yaw=0.0)]


def demo():
    v = os.environ.get("TASK_DEMO", "demo")
    digits = [int(c) for c in NUMBER]
    if v == "naive":  # each digit turned only most of the way, stopping 25 degrees short of the finger stop
        return {"left": [w for d in digits for w in dial(d, short_deg=25.0)] + HOME}
    if v == "unsafe":  # each digit turned to the stop, but the finger rides the wheel back
        return {"left": [w for d in digits for w in dial(d, ride=True)] + HOME}
    return {"left": [w for d in digits for w in dial(d)] + HOME}
