"""At the station table, a delivery note lies on the desk with an empty "date received" box printed on it, and a
self-inking date stamp stands beside it. The instruction asks for the note to be date-stamped.

A self-inking stamp is used the same way in every office, post room, library and school: hold it by its handle, set
its foot down square on the paper with the date the right way up, press the handle all the way down (the die flips
from the ink pad onto the paper only at the bottom of the stroke), let it spring back, lift it off and put it down.
Three everyday ways to spoil the form are physical: a half-hearted press that leaves no (or a faint) imprint, a
tilted press that smudges it, and an imprint upside down or outside the box. Stamping twice spoils it too.

The stamp is a loose frame (task-local bodykit.py) with its handle on a sprung slider: 15 mm of travel, the imprint is
made when the handle reaches the last 2 mm. The scorer records where the frame's foot stood, how it was turned and
how far it was tilted at that moment; that is the imprint.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Date-stamp the delivery note."
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bodykit import Body, BoxShape, as_numpy, on_reset, on_step, rotation, slider  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Date-stamp the delivery note."
HZ = 20.0

WHITE, PAPER, BLACK, GREY = (0.96, 0.96, 0.94), (0.97, 0.96, 0.92), (0.05, 0.05, 0.06), (0.30, 0.30, 0.32)
RED, BLUE, INK = (0.80, 0.12, 0.10), (0.20, 0.35, 0.75), (0.15, 0.20, 0.55)

# -- the delivery note: A5 (14.8 x 21 cm), lying loose; the "date received" box on its right half
N_XY, N_SIZE, N_T = (0.37, 0.12), (0.148, 0.210), 0.001
BOX_LOCAL = (-0.025, -0.045)  # the box's centre on the note (x toward the far edge, y toward the robot's left)
BOX_SIZE = (0.042, 0.074)  # 4.2 x 7.4 cm: the stamp's 3.0 x 6.0 cm imprint fits with 6-7 mm to spare
LINE = 0.0015

# -- the stamp (a Trodat 4810-sized date stamp): frame 6.4 x 3.2 cm, foot 6.0 x 3.0 cm, 8 cm tall at rest
S_XY = (0.24, 0.27)
F_L, F_W = 0.064, 0.032  # frame length (along its own x, the date's reading direction) and width
FOOT_H, SIDE_H = 0.006, 0.032
H_Z0 = 0.050  # the handle cap's underside above the foot's underside at rest (3 mm above the side plates when pressed)
CAP = (0.068, 0.036, 0.022)
TRAVEL = 0.015
INKED_AT = TRAVEL - 0.002  # the die reaches the paper in the last 2 mm


def _digit(d: int, cx: float, cy: float, h: float, z: float, up_deg: float) -> list:
    """A seven-segment digit ``h`` tall centred at (cx, cy), its top toward ``up_deg`` (0 = +x, 90 = +y)."""
    seg = {0: "abcdef", 1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc", 5: "afgcd", 6: "afgedc", 7: "abc",
           8: "abcdefg", 9: "abcdfg"}[d]
    w, t = 0.55 * h, 0.16 * h
    s = {"a": (h / 2, 0, w, t), "g": (0, 0, w, t), "d": (-h / 2, 0, w, t),
         "f": (h / 4, w / 2, t, h / 2), "b": (h / 4, -w / 2, t, h / 2),
         "e": (-h / 4, w / 2, t, h / 2), "c": (-h / 4, -w / 2, t, h / 2)}
    a = math.radians(up_deg)
    ca, sa = math.cos(a), math.sin(a)
    out = []
    for k in seg:
        ux, uy, sy, sx = s[k]
        size = (sx if k in "fbec" else t, sy if k in "agd" else t, 0.0004)
        out.append(BoxShape((cx + ca * ux - sa * uy, cy + sa * ux + ca * uy, z), size, BLACK, collide=False,
                            yaw=a))
    return out


DATE = "211126"


def _date_row(cx: float, cy: float, h: float, z: float, up_deg: float, pitch: float) -> list:
    """The date as six digits reading along ``up_deg - 90`` (left to right for a reader facing ``up_deg``)."""
    a = math.radians(up_deg - 90.0)
    out = []
    for i, ch in enumerate(DATE):
        off = (i - 2.5) * pitch + (0.0015 if i >= 2 else 0.0) + (0.0015 if i >= 4 else 0.0)
        out += _digit(int(ch), cx + off * math.cos(a), cy + off * math.sin(a), h, z, up_deg)
    return out


def _note() -> Body:
    sx, sy = N_SIZE
    bx, by = BOX_LOCAL
    lx, ly = BOX_SIZE
    z = N_T + 0.0002
    shapes = [BoxShape((0.0, 0.0, N_T / 2), (sx, sy, N_T), PAPER),
              BoxShape((sx / 2 - 0.016, 0.0, z), (0.012, sy - 0.03, 0.0004), BLUE, collide=False)]  # the header band
    for i in range(5):  # the address and item lines
        shapes.append(BoxShape((sx / 2 - 0.036 - 0.012 * i, 0.035, z), (LINE / 2, 0.11, 0.0004), GREY, collide=False))
    for s in (-1, 1):  # the box's outline
        shapes.append(BoxShape((bx + s * lx / 2, by, z), (LINE, ly + LINE, 0.0004), BLACK, collide=False))
        shapes.append(BoxShape((bx, by + s * ly / 2, z), (lx + LINE, LINE, 0.0004), BLACK, collide=False))
    shapes.append(BoxShape((bx + lx / 2 + 0.005, by, z), (0.003, ly, 0.0004), GREY, collide=False))  # "received"
    return Body.build("note", pos=(*N_XY, TOP), mass=0.005, shapes=shapes, color=PAPER, friction=(0.7, 0.6))


def _frame() -> Body:
    shapes = [BoxShape((0.0, 0.0, FOOT_H / 2), (F_L, F_W, FOOT_H), GREY),  # the foot round the die
              BoxShape((0.0, 0.0, 0.0006), (F_L - 0.008, F_W - 0.006, 0.0008), INK, collide=False)]  # the die
    for s in (-1, 1):  # the two side plates the handle runs between
        shapes.append(BoxShape((s * (F_L / 2 - 0.002), 0.0, SIDE_H / 2), (0.004, F_W, SIDE_H), GREY))
    return Body.build("stamp_frame", pos=(*S_XY, TOP), mass=0.045, shapes=shapes, color=GREY, friction=(0.8, 0.7),
                      com=(0.0, 0.0, 0.012))


def _handle() -> Body:
    shapes = [BoxShape((0.0, 0.0, CAP[2] / 2), CAP, BLACK),  # the handle cap
              BoxShape((0.0, 0.0, -0.012), (F_L - 0.012, F_W - 0.010, 0.024), BLACK),  # its stem inside the frame
              BoxShape((0.0, 0.0, CAP[2] + 0.0002), (0.054, 0.022, 0.0004), WHITE, collide=False)]  # the date window
    shapes += _date_row(0.0, 0.0, 0.010, CAP[2] + 0.0005, 90.0, 0.0075)  # the date as it will print, read along -x
    return Body.build("stamp_handle", pos=(S_XY[0], S_XY[1], TOP + H_Z0), mass=0.030, shapes=shapes, color=BLACK,
                      friction=(0.9, 0.8))


NOTE, FRAME, HANDLE = _note(), _frame(), _handle()
OBJECTS = [NOTE, FRAME, HANDLE]
PRESS_K, PRESS_PRELOAD = 80.0, 0.005  # N/m; the spring is preloaded 5 mm: 0.4 N at rest, 1.6 N at the bottom
slider("press", "stamp_frame", "stamp_handle", anchor=(S_XY[0], S_XY[1], TOP + H_Z0), axis="Z",
       limits=(-TRAVEL, 0.0), stiffness=PRESS_K, damping=2.0, target=PRESS_PRELOAD, max_force=50.0)

# -- scoring
RIGHT_YAW = -90.0  # the stamp's +x (the date's reading direction) along the robot's right (-y): the date upright
YAW_TOL = 8.0
IN_BOX_M = 0.007  # the imprint's centre within this of the box's centre, along each axis
TILT_MAX = 4.0  # tilted more than this at the bottom of the stroke: a smudged imprint
ON_PAPER_M = 0.003  # the foot's underside within this of the note's top
RESET_AT = 0.005  # the handle back above this before another stroke counts
CLEAR_M = 0.03
SUCCESS_HOLD_S = 1.0
PAD = 0.013


def _fresh() -> dict:
    return {"imprints": [], "down": False, "travel": 0.0, "max_travel": 0.0, "violated": False, "why": "",
            "held": 0, "step": 0, "tcps": None}


EP = _fresh()


@on_reset
def _new_episode() -> None:
    EP.clear()
    EP.update(_fresh())


def _yaw_deg(r) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _tilt_deg(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def imprint_quality(imp: dict) -> str:
    """'good' (clean, in the box, upright), 'upside down', 'askew', 'outside' or 'smudged'."""
    if not imp["on_paper"] or imp["tilt"] > TILT_MAX:
        return "smudged" if imp["on_paper"] else "off the note"
    if abs(imp["bx"]) > IN_BOX_M or abs(imp["by"]) > IN_BOX_M:
        return "outside"
    d = (imp["yaw"] - RIGHT_YAW + 180.0) % 360.0 - 180.0
    if abs(d) <= YAW_TOL:
        return "good"
    if abs(abs(d) - 180.0) <= YAW_TOL:
        return "upside down"
    return "askew"


def track(frame_pos, frame_rot, handle_pos, note_pos, note_rot, tcps) -> None:
    """Called once per control step with the stamp frame's and handle's poses, the note's pose and the two TCPs."""
    EP["step"] += 1
    EP["tcps"] = tcps
    fr = np.asarray(frame_rot, float)
    rel = fr.T @ (np.asarray(handle_pos, float) - np.asarray(frame_pos, float))
    travel = float(H_Z0 - rel[2])
    EP["travel"] = travel
    EP["max_travel"] = max(EP["max_travel"], travel)
    if not EP["down"] and travel >= INKED_AT:
        nr = np.asarray(note_rot, float)
        foot = np.asarray(frame_pos, float)
        local = nr[:2, :2].T @ (foot[:2] - np.asarray(note_pos, float)[:2])
        imp = {"step": EP["step"], "bx": round(float(local[0] - BOX_LOCAL[0]), 4),
               "by": round(float(local[1] - BOX_LOCAL[1]), 4),
               "yaw": round(_yaw_deg(fr) - _yaw_deg(nr), 1), "tilt": round(_tilt_deg(fr), 1),
               "on_paper": bool(abs(float(foot[2]) - (float(note_pos[2]) + N_T)) < ON_PAPER_M)}
        imp["quality"] = imprint_quality(imp)
        EP["imprints"].append(imp)
        EP["down"] = True
        if len(EP["imprints"]) > 1:
            _violate("stamped twice: the note is spoilt")
    elif EP["down"] and travel < RESET_AT:
        EP["down"] = False
    good = len(EP["imprints"]) == 1 and EP["imprints"][0]["quality"] == "good"
    done = good and not EP["down"] and _put_down(frame_pos, fr, tcps)
    EP["held"] = EP["held"] + 1 if done else 0


def _violate(why: str) -> None:
    if not EP["violated"]:
        EP["violated"], EP["why"] = True, f"{why} at step {EP['step']}"


def _put_down(frame_pos, fr, tcps) -> bool:
    """The stamp let go and standing: upright, its foot on the desk or the note, every pad tip 3 cm from it."""
    if _tilt_deg(fr) > 10.0 or float(frame_pos[2]) > TOP + N_T + 0.004:
        return False
    if tcps is None:
        return True
    c = np.asarray(frame_pos, float) + fr @ np.array([0.0, 0.0, H_Z0 / 2])
    half = np.array([CAP[0] / 2, CAP[1] / 2, H_Z0 / 2 + CAP[2]])
    for t in tcps:
        tip = np.asarray(t, float) - np.array([0.0, 0.0, PAD])
        local = np.abs(fr.T @ (tip - c))
        if np.all(local < half + CLEAR_M):
            return False
    return True


def _tcps(base, origin) -> list:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


_DEBUG = os.environ.get("HH_DEBUG")


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    fp = as_numpy(base.scene["stamp_frame"].data.root_link_pose_w)[0]
    hp = as_numpy(base.scene["stamp_handle"].data.root_link_pose_w)[0]
    npose = as_numpy(base.scene["note"].data.root_link_pose_w)[0]
    track(fp[:3] - origin, rotation(fp[3:7]), hp[:3] - origin, npose[:3] - origin, rotation(npose[3:7]),
          _tcps(base, origin))
    if _DEBUG and EP["step"] % 5 == 0:
        import json
        rec = {"step": EP["step"], "grippers": [{"tcp": [round(float(v), 4) for v in t]} for t in EP["tcps"]],
               "frame": [round(float(v), 4) for v in (fp[:3] - origin)], "yaw": round(_yaw_deg(rotation(fp[3:7])), 1),
               "tilt": round(_tilt_deg(rotation(fp[3:7])), 1),
               "note": [round(float(v), 4) for v in (npose[:3] - origin)],
               "mem": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in EP.items()
                       if isinstance(v, (int, float, str, bool, list)) and k != "tcps"}}
        with open(_DEBUG, "a") as f:
            f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    return not EP["violated"] and EP["held"] >= int(SUCCESS_HOLD_S * HZ)


def score(objects) -> float:
    """1.0: one clean, upright imprint in the box and the stamp put down (held 1 s). Otherwise partial credit for
    the imprint: 0.5 for a good one with the stamp not yet put down, 0.25 for a clean one upside down, askew or outside
    the box. 0 for no imprint, a smudged one, or stamping twice."""
    if EP["violated"] or not EP["imprints"]:
        return 0.0
    if EP["held"] >= int(SUCCESS_HOLD_S * HZ):
        return 1.0
    q = EP["imprints"][0]["quality"]
    if q == "good":
        return 0.5
    if q in ("upside down", "askew", "outside"):
        return 0.25
    return 0.0


def distance(objects) -> float:
    return max(0.0, INKED_AT - EP["max_travel"]) + (0.0 if EP["imprints"] else 0.01)


# -- scripted plans (left arm): grip the handle's cap, turn the stamp a quarter turn, press it in the box ------------
REST = (0.20, 0.30, TOP + 0.20)
GRIP_TIP = TOP + H_Z0 + CAP[2] - 0.011  # the pad tips 11 mm down the cap's sides
LIFT_TIP = GRIP_TIP + 0.06


def _box_world() -> tuple:
    return (N_XY[0] + BOX_LOCAL[0], N_XY[1] + BOX_LOCAL[1])


GRIP_OFF = (0.0028, -0.0156)  # where the stamp sits relative to the TCP once gripped (yaw 0; one jaw closes further)


def _held_at(x: float, y: float, yaw: float) -> tuple:
    """The TCP position that puts a gripped stamp's centre at (x, y) with the gripper at ``yaw``."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    ox, oy = GRIP_OFF
    return (x - (c * ox - s_ * oy), y - (s_ * ox + c * oy))


def plan(press: float = TRAVEL + 0.003, yaw_deg: float = RIGHT_YAW) -> list:
    """Grip the cap, lift, turn the stamp the right way round over the box, set it down square, press the handle to the
    bottom still gripping it, let it spring back, lift it off and stand it back beside the note."""
    x, y = S_XY
    bx, by = _box_world()
    yaw = math.radians(yaw_deg)  # at yaw 0 the pads close along y, across the stamp's width: gripper yaw = stamp yaw
    hx, hy = _held_at(bx, by, yaw)
    sx, sy = _held_at(x, y, yaw)
    seat = GRIP_TIP + N_T  # gripping height with the foot on the note
    return [Waypoint(1.5, (x, y, LIFT_TIP + 0.02), FINGER_OPEN, "above the stamp", yaw=0.0),
            Waypoint(1.0, (x, y, GRIP_TIP + PAD), FINGER_OPEN, "pads either side of the handle", yaw=0.0),
            Waypoint(0.8, (x, y, GRIP_TIP + PAD), FINGER_CLOSED, "grip the handle", yaw=0.0),
            Waypoint(1.0, (x, y, LIFT_TIP + PAD), FINGER_CLOSED, "lift the stamp", yaw=0.0),
            Waypoint(2.0, (hx, hy, LIFT_TIP + PAD), FINGER_CLOSED, "over the box, the date the right way up", yaw=yaw),
            Waypoint(1.2, (hx, hy, seat + 0.004 + PAD), FINGER_CLOSED, "foot just over the note", yaw=yaw),
            Waypoint(0.6, (hx, hy, seat + PAD), FINGER_CLOSED, "foot down on the note", yaw=yaw),
            Waypoint(1.0, (hx, hy, seat - press + PAD), FINGER_CLOSED, "press the handle down", yaw=yaw),
            Waypoint(0.4, (hx, hy, seat - press + PAD), FINGER_CLOSED, "hold at the bottom", yaw=yaw),
            Waypoint(0.8, (hx, hy, seat + PAD), FINGER_CLOSED, "let it spring back", yaw=yaw),
            Waypoint(1.0, (hx, hy, LIFT_TIP + PAD), FINGER_CLOSED, "lift it off", yaw=yaw),
            Waypoint(2.0, (sx, sy, LIFT_TIP + PAD), FINGER_CLOSED, "back beside the note", yaw=yaw),
            Waypoint(1.0, (sx, sy, GRIP_TIP + 0.002 + PAD), FINGER_CLOSED, "stand it on the desk", yaw=yaw),
            Waypoint(0.6, (sx, sy, GRIP_TIP + 0.002 + PAD), FINGER_OPEN, "let go", yaw=yaw),
            Waypoint(0.8, (sx, sy, LIFT_TIP + 0.04 + PAD), FINGER_OPEN, "up off it", yaw=yaw)]


HOME = [Waypoint(1.5, REST, FINGER_OPEN, "back to rest", yaw=0.0)]


def demo():
    v = os.environ.get("TASK_DEMO", "demo")
    if v == "naive":  # the stamp set down in the box and pressed only half way: the die never reaches the paper
        return {"left": plan(press=0.008) + HOME}
    if v == "unsafe":  # pressed fully, but the stamp turned the other way: the date printed upside down
        return {"left": plan(yaw_deg=RIGHT_YAW + 180.0) + HOME}
    return {"left": plan() + HOME}
