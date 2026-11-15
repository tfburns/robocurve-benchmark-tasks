"""hhperson: a static, realistic person (a posed mannequin) for a yam_arena task file.

The person is a Microsoft Rocketbox avatar (MIT licence, https://github.com/microsoft/Microsoft-Rocketbox), posed
once offline (standing at the bench, leaning on it, hands resting flat) and baked into a static textured mesh:
``assets/person/<name>.usdc``. It never moves. In the simulator it is one kinematic body whose collision shapes are
invisible capsules (one per arm segment, hand, torso, head and leg), so the arms are physically blocked by it; the
visual mesh has no collider. ``<name>_proxies.json`` holds the same capsules and each hand's outline seen from above,
which the task's scorer uses.

yam_arena's task files spawn primitives and library assets only. Importing this module adds :class:`UsdBody` (a
fixed body loaded from a USD file in this folder) by hooking ``yam_arena.scene`` when it is imported, without changing
yam_arena (the same approach as the accepted lock-before-loading task's scenekit). Pure Python at import time.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

import hhkit

HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class UsdBody(Box):
    """A fixed (kinematic) body from ``usd`` (relative to this folder), its file origin at ``pos``. ``size`` is
    nominal (the bounding box, for the task-file checks)."""

    usd: str = ""

    def _check(self) -> None:
        if not (HERE / self.usd).is_file():
            raise FileNotFoundError(f"{self.name}: {HERE / self.usd} not found")
        if self.mass is not None:
            raise ValueError(f"{self.name}: a UsdBody here is fixed (mass=None)")


class Person:
    """A posed person: the USD body for OBJECTS and its collision capsules and hand outlines for scoring."""

    def __init__(self, name: str = "colleague", folder: str = "assets/person", stem: str = "colleague"):
        self.name = name
        data = json.loads((HERE / folder / f"{stem}_proxies.json").read_text())
        self.capsules = {c["name"]: (np.asarray(c["a"], float), np.asarray(c["b"], float), float(c["r"]))
                         for c in data["capsules"]}
        self.outlines = {k: np.asarray(v, float) for k, v in data["outlines"].items() if not k.endswith("_min_z")}
        self.points = {k: np.asarray(v, float) for k, v in data.get("points", {}).items()}  # skin samples per part
        self.source = data["source"]
        lo, hi = (np.asarray(v, float) for v in data["bounds"])
        self.bounds = (lo, hi)
        self.spec = UsdBody(name, pos=(0.0, 0.0, 0.0), size=tuple(float(v) for v in hi - lo),
                            color=(0.8, 0.7, 0.6), usd=f"{folder}/{stem}.usdc")

    def parts(self, *names: str) -> list:
        return [self.capsules[n] for n in names] if names else list(self.capsules.values())

    def robot_gap(self, robot_caps, parts=("R_hand", "R_forearm")) -> float:
        """Smallest gap (m) between any robot capsule and the skin of the named parts (sampled skin points)."""
        if not robot_caps:
            return float("inf")
        pts = np.vstack([self.points[p] for p in parts])
        return float(min(_points_capsule(pts, c).min() for c in robot_caps))

    def point_gap(self, p, parts=()) -> float:
        return min(hhkit.point_capsule(p, m) for m in self.parts(*parts))


def _points_capsule(P, cap) -> np.ndarray:
    a, b, r = np.asarray(cap[0], float), np.asarray(cap[1], float), float(cap[2])
    ab = b - a
    t = np.clip(((P - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1) - r


# -- the simulator side: spawn UsdBody objects and report their fixed state -------------------------------------
_SPECS: dict[str, UsdBody] = {}


def _usd_object(spec: UsdBody):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
             "physics_material": sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.7)}
    obj = Object(name=spec.name, usd_path=str(HERE / spec.usd), object_type=ObjectType.RIGID, spawn_cfg_addon=addon)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))
    return obj


def _state(spec: UsdBody) -> ObjectState:
    pos = np.asarray(spec.pos, float)
    half = np.asarray(spec.size, float) / 2
    return ObjectState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - half, high=pos + half)


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhperson", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def task_object_hook(spec):
        if isinstance(spec, UsdBody):
            _SPECS[spec.name] = spec
            return _usd_object(spec)
        return task_object(spec)

    def object_states_hook(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(_SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object_hook, object_states_hook
    scene._hhperson = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches yam_arena.scene as it is imported (it needs Isaac Sim, which starts after the task file loads)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
