"""Offline check of the scorer on kinematic replays of the scripted plans (no simulator): the demo scores 1.0, the
hand-to-hand pass, the blade-first lay and the drop 0.0."""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

M = load(str(pathlib.Path(__file__).resolve().parent / "task.py")).module
GEOMS = {"scalpel": R.Geom("box", np.array(M.SCALPEL) / 2), "forceps": R.Geom("box", np.array(M.FORCEPS) / 2)}


def run(name):
    M.TRIAL.clear()
    frames = R.replay(M, M.PLANS[name](), links=True, extra_s=2.0, geoms=GEOMS)
    z = [float(f[0]["scalpel"].pos[2]) for f in frames]
    for k in range(1, len(frames)):  # the replay snaps a released load onto its support: let it fall instead
        if z[k - 1] - z[k] > 0.004:
            t, z0, j = 0.0, z[k - 1], k
            while j < len(frames):
                t += 1 / R.HZ
                zz = max(z[k], z0 - 0.5 * 9.81 * t * t)
                st = frames[j][0]["scalpel"]
                v = np.array([0.0, 0.0, -9.81 * t if zz > z[k] else 0.0])
                frames[j][0]["scalpel"] = R.state(np.array([*st.pos[:2], zz]), st.quat_xyzw, GEOMS["scalpel"].half, v)
                if zz <= z[k]:
                    break
                j += 1
    return R.run(M, M.TRIAL, frames), M.TRIAL.mem.get("why", ""), {k: round(v, 3) if isinstance(v, float) else v
                                                                   for k, v in M.TRIAL.mem.items()}


if __name__ == "__main__":
    for n in M.PLANS:
        print(n, *run(n))
