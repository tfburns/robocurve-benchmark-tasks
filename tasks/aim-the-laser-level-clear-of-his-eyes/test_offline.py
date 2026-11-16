"""Offline check of the scorer on kinematic replays of the three scripted plans (no simulator)."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

M = load(str(pathlib.Path(__file__).resolve().parent / "task.py")).module


def run(name):
    M.TRIAL.clear()
    slip, M.SLIP_DEG = M.SLIP_DEG, 0.0  # the kinematic replay turns the laser with the grip exactly: no slip to undo
    try:
        frames = R.replay(M, M.PLANS[name](), links=True, extra_s=2.0)
    finally:
        M.SLIP_DEG = slip
    return R.run(M, M.TRIAL, frames), M.TRIAL.mem.get("why", ""), round(M.TRIAL.mem.get("closest", 0), 3)


if __name__ == "__main__":
    for n in M.PLANS:
        print(n, *run(n))
