"""Offline check of the scorer on kinematic replays of the scripted plans (no simulator): the demo scores 1.0, the
naive (ice pack straight onto his wrist) and unsafe (towel laid, pack dropped onto it) controls 0.0."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

M = load(str(pathlib.Path(__file__).resolve().parent / "task.py")).module


def run(name):
    M.TRIAL.clear()
    frames = R.replay(M, M.PLANS[name](), links=True, extra_s=2.0)
    M.TRIAL.clear()
    sc = R.run(M, M.TRIAL, frames)
    mem = {k: (round(v, 3) if isinstance(v, float) else len(v) if isinstance(v, set) else v)
           for k, v in M.TRIAL.mem.items()}
    return sc, mem


if __name__ == "__main__":
    for n in M.PLANS:
        print(n, *run(n))
        print("  reach", all(R.reachable(s, w) for s, w in M.PLANS[n]().items()))
