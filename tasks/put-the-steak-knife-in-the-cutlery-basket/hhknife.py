"""hhknife: a steak knife as one rigid body for a yam_arena task file.

:class:`Knife` is a handle (a box, spawned the way yam_arena spawns its own box) with the blade added as a second box
collider and look on the same rigid body, as the bottle in our hhcarry kit adds its neck and cap. In the knife's own
frame the handle is centred on the origin along x and the blade runs from the handle's -x end out to the tip at
-x. Importing this module hooks ``yam_arena.scene`` so a Knife in ``OBJECTS`` is spawned this way; its state is
yam_arena's own rigid-object state (``pos`` is the centre of the whole knife's bounding box). yam_arena is not
modified.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box


@dataclass(frozen=True)
class Knife(Box):
    """``pos`` is the handle's centre; ``size`` is the handle (length, width, thickness)."""

    blade: tuple = (0.11, 0.020, 0.003)  # length, width, thickness
    blade_color: tuple = (0.80, 0.81, 0.83)
    friction: tuple = (0.8, 0.7)

    @property
    def local_centre(self) -> np.ndarray:
        """The bounding-box centre in the knife's own frame (what yam_arena reports as ``pos``)."""
        hl, bl = self.size[0], self.blade[0]
        return np.array([(-(hl / 2 + bl) + hl / 2) / 2, 0.0, 0.0])

    @property
    def length(self) -> float:
        return self.size[0] + self.blade[0]


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def handle_centre(spec: Knife, state) -> np.ndarray:
    return np.asarray(state.pos, float) - _rot(state.quat_xyzw) @ spec.local_centre


def points(spec: Knife, state) -> dict:
    """World points along the knife: tip, heel (blade meets handle) and butt (handle end); and the blade's direction
    (unit, from the heel toward the tip)."""
    R, h = _rot(state.quat_xyzw), handle_centre(spec, state)
    hl, bl = spec.size[0], spec.blade[0]
    heel = h + R @ np.array([-hl / 2, 0.0, 0.0])
    tip = h + R @ np.array([-(hl / 2 + bl), 0.0, 0.0])
    butt = h + R @ np.array([hl / 2, 0.0, 0.0])
    return {"tip": tip, "heel": heel, "butt": butt, "handle": h, "dir": R @ np.array([-1.0, 0.0, 0.0])}


def _knife_object(spec: Knife, scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material, clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    hl, _hw, ht = spec.size
    bl, bw, bt = spec.blade

    @clone
    def _spawn(prim_path, cfg, *args, **kwargs):
        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        phys = f"{prim_path}/Looks/blade_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
        pm.func(phys, pm)
        look = f"{prim_path}/Looks/steel"
        lk = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.blade_color), roughness=0.25, metallic=0.9)
        lk.func(look, lk)
        g = UsdGeom.Cube.Define(stage, f"{prim_path}/blade")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(-(hl / 2 + bl / 2), 0.0, -ht / 2 + bt / 2))
        g.AddScaleOp().Set(Gf.Vec3f(bl, bw, bt))
        p = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(p)
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(p.GetPath()), phys, stage=stage)
        bind_visual_material(str(p.GetPath()), look, stage=stage)
        return prim

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size), func=_spawn,
        rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0],
                                                        dynamic_friction=spec.friction[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.5))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhknife", False):
        return
    orig_task_object = scene.task_object

    def task_object(spec):
        if isinstance(spec, Knife):
            return _knife_object(spec, scene)
        return orig_task_object(spec)

    scene.task_object, scene._hhknife = task_object, True


class _Finder(importlib.abc.MetaPathFinder):
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
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
