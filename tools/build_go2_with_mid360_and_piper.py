#!/usr/bin/env python3
# Rebuilds data/Robots/Go2/usd/go2_with_mid360_and_piper.usd from
# go2_with_mid360.usd + data/Robots/Piper/usd/piper.usd (a Piper arm,
# sourced from https://github.com/agilexrobotics/piper_isaac_sim's
# piper_description_v100_realsense_camera_v2 -- see README.md for
# provenance/attribution), welded to the chassis by the arm's own
# root_joint (a PhysicsFixedJoint carrying its ArticulationRootAPI, shipped
# welding the arm to "world" -- redirected here to Go2's base instead, at
# the mount offset/tilt).
#
# The arm stays a *second*, independent PhysX articulation (its own
# ArticulationRootAPI), not merged into the chassis's -- this is exactly the
# mechanism piper.usd's own root_joint already exists for, and it isn't
# optional here: Go2FlatTerrainPolicy indexes robot.get_joint_positions()
# positionally against its checkpoint's own 12-leg-joint arrays (see go2.py),
# so folding Piper's 8 DOFs into that same articulation would silently
# misalign every observation/action term. PhysX also flatly refuses to nest
# one articulation root under another rigid body that's already part of an
# articulation, which is why the Piper mount prim below is a *sibling* of
# Go2's `base` (which carries Go2's own ArticulationRootAPI), not a child of
# it -- nesting it there was tried first and silently aborted building
# Piper's entire articulation (cascading into "no bodies defined" for every
# one of its internal joints).
#
# Run with Isaac Sim's own Python (needs the isaacsim package + pxr):
#   ./python.sh tools/build_go2_with_mid360_and_piper.py
# from the Isaac Sim install directory, or via the isaac_run/isaac_go2_run
# shell function documented in docs/README.md:
#   isaac_run tools/build_go2_with_mid360_and_piper.py
#
# Re-run this any time piper.usd changes, or to move the mount -- it
# preserves the existing file's Piper mount Xform's transform (translate/
# rotate -- the real-world mount offset/tilt, measured against the physical
# robot) if the file already exists, so you don't need to remember or
# hardcode those numbers here. The shipped default is a rough "somewhere on
# the back" placeholder -- measure your actual bracket and adjust via the
# Isaac Sim UI's Transform properties on the Piper mount prim, same workflow
# as the Mid-360 mount (see tools/build_go2_with_mid360.py).
#
# If something goes wrong, set BUILD_PIPER_DEBUG_LOG=/some/path.log before
# running for a step-by-step trace written straight to a file -- plain
# print() output from a script like this is not reliably captured when
# piping Isaac Sim's headless stdout (Kit's own logging pipeline can drop
# unflushed buffers on shutdown, which cost real debugging time building
# the Mid-360 asset).

import os
import sys

_DEBUG_LOG = os.environ.get("BUILD_PIPER_DEBUG_LOG")


def _debug(msg):
    if _DEBUG_LOG:
        with open(_DEBUG_LOG, "a") as f:
            f.write(str(msg) + "\n")


from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})
_debug("simulation app created")

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, EXT_ROOT)

from isaacsim.core.utils.extensions import enable_extension

enable_extension("go2_in_isaacsim")
simulation_app.update()

OUTPUT_PATH = os.path.join(EXT_ROOT, "data", "Robots", "Go2", "usd", "go2_with_mid360_and_piper.usd")
GO2_MID360_REF = "./go2_with_mid360.usd"
PIPER_REF = "../../Piper/usd/piper.usd"
BASE_PATH = "/go2_description/base"
# NOT a child of BASE_PATH: /go2_description/base itself carries Go2's own
# PhysicsArticulationRootAPI (its chassis+legs articulation), and PhysX
# forbids nesting one articulation root under another rigid body that's
# already part of an articulation ("UsdPhysics: Nested articulation roots
# are not allowed" -- discovered by trying that first: it silently aborted
# building the *whole* Piper articulation, cascading into "no bodies
# defined" for every one of its internal joints too). A sibling of base
# under /go2_description keeps Piper a second, independent PhysX
# articulation -- see the module docstring for why that matters here
# (Go2's own policy indexes robot.get_joint_positions() positionally, so
# merging Piper's DOFs into the same articulation would silently corrupt
# its action/observation arrays).
MOUNT_PATH = "/go2_description/Piper"
ROOT_JOINT_PATH = f"{MOUNT_PATH}/root_joint"
# Rough placeholder: centered, up on the back. Not a measurement.
DEFAULT_MOUNT_TRANSLATE = Gf.Vec3d(0.0, 0.0, 0.15)
DEFAULT_MOUNT_ROTATE_ZYX = Gf.Vec3d(0.0, 0.0, 0.0)


def _read_existing_mount_transform():
    """If go2_with_mid360_and_piper.usd already exists, read the Piper mount
    Xform's current translate/rotateZYX ops (the real-world offset/tilt
    someone measured against the physical robot) so rebuilding doesn't
    silently discard that adjustment."""
    if not os.path.isfile(OUTPUT_PATH):
        return DEFAULT_MOUNT_TRANSLATE, DEFAULT_MOUNT_ROTATE_ZYX
    stage = Usd.Stage.Open(OUTPUT_PATH)
    prim = stage.GetPrimAtPath(MOUNT_PATH)
    if not prim.IsValid():
        return DEFAULT_MOUNT_TRANSLATE, DEFAULT_MOUNT_ROTATE_ZYX
    translate_attr = prim.GetAttribute("xformOp:translate")
    rotate_attr = prim.GetAttribute("xformOp:rotateZYX")
    translate = translate_attr.Get() if translate_attr.IsValid() else DEFAULT_MOUNT_TRANSLATE
    rotate = rotate_attr.Get() if rotate_attr.IsValid() else DEFAULT_MOUNT_ROTATE_ZYX
    return translate, rotate


def main():
    mount_translate, mount_rotate_zyx = _read_existing_mount_transform()
    print(f"Piper mount transform: translate={mount_translate}  rotateZYX={mount_rotate_zyx}")
    _debug(f"Piper mount transform: translate={mount_translate}  rotateZYX={mount_rotate_zyx}")

    # See build_go2_with_mid360.py's comment on this same pattern: build into
    # a temp file next to the real one, only replace it on full success, so
    # a mid-build failure can never leave OUTPUT_PATH holding an empty
    # placeholder stage instead of its previous (possibly hand-adjusted,
    # uncommitted) content.
    temp_path = OUTPUT_PATH + ".building.usd"

    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    simulation_app.update()

    assert usd_context.save_as_stage(temp_path)
    simulation_app.update()
    stage = usd_context.get_stage()

    robot_prim = stage.DefinePrim("/go2_description", "Xform")
    robot_prim.GetReferences().AddReference(GO2_MID360_REF)
    stage.SetDefaultPrim(robot_prim)
    simulation_app.update()
    simulation_app.update()
    base_prim = stage.GetPrimAtPath(BASE_PATH)
    assert base_prim.IsValid(), "go2_with_mid360.usd reference did not compose 'base'"

    # Reference piper.usd *directly* onto the mount prim (not a child of
    # it): piper.usd's defaultPrim ("/piper_camera") composes its own
    # children (arm_base, link1..8, joints/, root_joint, ...) onto whatever
    # prim references it -- same reasoning as Mid360.usd's own mount
    # (see build_go2_with_mid360.py).
    mount_prim = stage.DefinePrim(MOUNT_PATH, "Xform")
    mount_prim.GetReferences().AddReference(PIPER_REF)
    mount_xformable = UsdGeom.Xformable(mount_prim)
    mount_xformable.ClearXformOpOrder()
    mount_xformable.AddTranslateOp().Set(mount_translate)
    mount_xformable.AddRotateZYXOp().Set(mount_rotate_zyx)
    simulation_app.update()

    root_joint = stage.GetPrimAtPath(ROOT_JOINT_PATH)
    assert root_joint.IsValid(), f"{ROOT_JOINT_PATH} not found -- did piper.usd's structure change?"
    joint_api = UsdPhysics.Joint(root_joint)
    assert joint_api, f"{root_joint.GetPath()} is not a UsdPhysics.Joint"
    _debug(f"root_joint body0 before: {joint_api.GetBody0Rel().GetTargets()}")

    # root_joint ships welding arm_base to "world" (body0 empty, localPos0/
    # localRot0 identity). Redirect it to weld to Go2's base instead, at the
    # mount prim's pose *relative to base* -- computed from each prim's own
    # composed world transform (rather than re-deriving translate+rotateZYX
    # -> quaternion by hand, or assuming the mount prim is base's scenegraph
    # child, which it deliberately isn't -- see MOUNT_PATH above) so the
    # physical attachment (this joint) and the visual one (the mount
    # Xform's own transform, for a sane look before physics ever runs) stay
    # consistent from one source of truth.
    # USD matrices are row-vector (p_world = p_local * localToWorld), so
    # "mount's pose expressed in base's frame" is mount_world * base_world^-1.
    base_xformable = UsdGeom.Xformable(base_prim)
    mount_relative_to_base = mount_xformable.ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    ) * base_xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    mount_local_translate = mount_relative_to_base.ExtractTranslation()
    mount_local_quat = mount_relative_to_base.ExtractRotationQuat()

    joint_api.GetBody0Rel().SetTargets([Sdf.Path(BASE_PATH)])
    # UsdPhysics Joint local frames are single-precision (point3f/quatf),
    # unlike general xformOps -- verified against this exact prim beforehand
    # to avoid a repeat of the AddOrientOp() float/double mismatch hit while
    # building the Mid-360 sensor (see that commit).
    joint_api.GetLocalPos0Attr().Set(Gf.Vec3f(mount_local_translate))
    imaginary = mount_local_quat.GetImaginary()
    joint_api.GetLocalRot0Attr().Set(
        Gf.Quatf(float(mount_local_quat.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2]))
    )
    _debug(f"root_joint body0 after: {joint_api.GetBody0Rel().GetTargets()}")
    _debug(f"root_joint localPos0 after: {joint_api.GetLocalPos0Attr().Get()}")
    _debug(f"root_joint localRot0 after: {joint_api.GetLocalRot0Attr().Get()}")
    print(f"root_joint welded: {ROOT_JOINT_PATH} body0 -> {BASE_PATH}")

    save_ok = usd_context.save_stage()
    _debug(f"save_stage() returned: {save_ok}")
    assert save_ok
    usd_context.close_stage()
    simulation_app.update()
    _debug(f"about to replace {temp_path} -> {OUTPUT_PATH}")
    os.replace(temp_path, OUTPUT_PATH)
    _debug("replace done")
    print(f"Saved {OUTPUT_PATH}")


try:
    main()
    _debug("main() completed without exception")
except Exception as e:
    _debug(f"main() raised: {type(e).__name__}: {e}")
    raise
finally:
    temp_path = OUTPUT_PATH + ".building.usd"
    if os.path.isfile(temp_path):
        _debug(f"finally: removing leftover temp file {temp_path}")
        os.remove(temp_path)
    else:
        _debug("finally: no leftover temp file")
    simulation_app.close()
    _debug("simulation_app closed")
