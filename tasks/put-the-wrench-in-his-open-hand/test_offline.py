"""Offline check of the scorer on kinematic replays of the three scripted plans (no simulator): the demo scores 1.0,
the drop and the rushed hand-over 0.0. The palm is stood in for by a fixed box the size of its collider slab."""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import Box, load  # noqa: E402

import hhreplay as R  # noqa: E402

M = load(str(pathlib.Path(__file__).resolve().parent / "task.py")).module
PALM = Box("palm_proxy", pos=tuple(M.PALM_C), size=tuple(M.PALM_S))
GEOMS = {"wrench": R.Geom("box", np.array([M.WRENCH[0], M.WRENCH[1], M.WRENCH[2]]) / 2)}  # lying along y


def run(name):
    objs = M.OBJECTS
    M.OBJECTS = [*objs, PALM]
    try:
        M.TRIAL.clear()
        frames = R.replay(M, M.PLANS[name](), links=True, extra_s=2.0, geoms=GEOMS)
        z = [float(f[0]["wrench"].pos[2]) for f in frames]
        for k in range(1, len(frames)):  # the replay snaps a released load onto its support: let it fall instead
            if z[k - 1] - z[k] > 0.004:
                t, z0 = 0.0, z[k - 1]
                j = k
                while j < len(frames):
                    t += 1 / R.HZ
                    zz = max(z[k], z0 - 0.5 * 9.81 * t * t)
                    st = frames[j][0]["wrench"]
                    v = np.array([0.0, 0.0, -9.81 * t if zz > z[k] else 0.0])
                    frames[j][0]["wrench"] = R.state(np.array([*st.pos[:2], zz]), st.quat_xyzw, GEOMS["wrench"].half, v)
                    if zz <= z[k]:
                        break
                    j += 1
        for f in frames:
            f[0].pop("palm_proxy", None)
        return R.run(M, M.TRIAL, frames), M.TRIAL.mem.get("why", "")
    finally:
        M.OBJECTS = objs


if __name__ == "__main__":
    for n in M.PLANS:
        print(n, *run(n))
